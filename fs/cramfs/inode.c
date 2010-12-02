/*
 * Compressed rom filesystem for Linux.
 *
 * Copyright (C) 1999 Linus Torvalds.
 *
 * This file is released under the GPL.
 */

/*
 * These are the VFS interfaces to the compressed rom filesystem.
 * The actual compression is based on zlib, see the other files.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/blkdev.h>
#include <linux/cramfs_fs.h>
#include <linux/slab.h>
#include <linux/cramfs_fs_sb.h>
#include <linux/buffer_head.h>
#include <linux/vfs.h>
#include <asm/semaphore.h>
#if CRAMFS_BADBLOCK
#include <linux/mtd/mtd.h>
#endif

#include <asm/uaccess.h>

#if CRAMFS_BADBLOCK
static unsigned int *bbTable;
static unsigned int bbTabMaxIndex = 0;
static unsigned int bbBlockSize = 0;
#endif

static struct super_operations cramfs_ops;
static struct inode_operations cramfs_dir_inode_operations;
static struct file_operations cramfs_directory_operations;
static struct address_space_operations cramfs_aops;

static DECLARE_MUTEX(read_mutex);


/* These two macros may change in future, to provide better st_ino
   semantics. */
#define CRAMINO(x)	(((x)->offset && (x)->size)?(x)->offset<<2:1)
#define OFFSET(x)	((x)->i_ino)


static int cramfs_iget5_test(struct inode *inode, void *opaque)
{
	struct cramfs_inode *cramfs_inode = opaque;

	if (inode->i_ino != CRAMINO(cramfs_inode))
		return 0; /* does not match */

	if (inode->i_ino != 1)
		return 1;

	/* all empty directories, char, block, pipe, and sock, share inode #1 */

	if ((inode->i_mode != cramfs_inode->mode) ||
	    (inode->i_gid != cramfs_inode->gid) ||
	    (inode->i_uid != cramfs_inode->uid))
		return 0; /* does not match */

	if ((S_ISCHR(inode->i_mode) || S_ISBLK(inode->i_mode)) &&
	    (inode->i_rdev != old_decode_dev(cramfs_inode->size)))
		return 0; /* does not match */

	return 1; /* matches */
}

static int cramfs_iget5_set(struct inode *inode, void *opaque)
{
	static struct timespec zerotime;
	struct cramfs_inode *cramfs_inode = opaque;
	inode->i_mode = cramfs_inode->mode;
	inode->i_uid = cramfs_inode->uid;
	inode->i_size = cramfs_inode->size;
	inode->i_blocks = (cramfs_inode->size - 1) / 512 + 1;
	inode->i_blksize = PAGE_CACHE_SIZE;
	inode->i_gid = cramfs_inode->gid;
	/* Struct copy intentional */
	inode->i_mtime = inode->i_atime = inode->i_ctime = zerotime;
	inode->i_ino = CRAMINO(cramfs_inode);
	/* inode->i_nlink is left 1 - arguably wrong for directories,
	   but it's the best we can do without reading the directory
           contents.  1 yields the right result in GNU find, even
	   without -noleaf option. */
	if (S_ISREG(inode->i_mode)) {
		inode->i_fop = &generic_ro_fops;
		inode->i_data.a_ops = &cramfs_aops;
	} else if (S_ISDIR(inode->i_mode)) {
		inode->i_op = &cramfs_dir_inode_operations;
		inode->i_fop = &cramfs_directory_operations;
	} else if (S_ISLNK(inode->i_mode)) {
		inode->i_op = &page_symlink_inode_operations;
		inode->i_data.a_ops = &cramfs_aops;
	} else {
		inode->i_size = 0;
		inode->i_blocks = 0;
		init_special_inode(inode, inode->i_mode,
			old_decode_dev(cramfs_inode->size));
	}
	return 0;
}

static struct inode *get_cramfs_inode(struct super_block *sb,
				struct cramfs_inode * cramfs_inode)
{
	struct inode *inode = iget5_locked(sb, CRAMINO(cramfs_inode),
					    cramfs_iget5_test, cramfs_iget5_set,
					    cramfs_inode);
	if (inode && (inode->i_state & I_NEW)) {
		unlock_new_inode(inode);
	}
	return inode;
}

#if CRAMFS_BADBLOCK
unsigned int cramfs_bb_xlat(unsigned int logicalAddr) {
  unsigned int lsbs;

  if( logicalAddr > bbTabMaxIndex * bbBlockSize ) {
    return logicalAddr; // we can't translate out of bound requests!
  }

  lsbs = logicalAddr & (bbBlockSize - 1);  // we assume pow2 for blocksize
  return( bbTable[(logicalAddr / bbBlockSize)]  | lsbs  );
}
#endif

/*
 * We have our own block cache: don't fill up the buffer cache
 * with the rom-image, because the way the filesystem is set
 * up the accesses should be fairly regular and cached in the
 * page cache and dentry tree anyway..
 *
 * This also acts as a way to guarantee contiguous areas of up to
 * BLKS_PER_BUF*PAGE_CACHE_SIZE, so that the caller doesn't need to
 * worry about end-of-buffer issues even when decompressing a full
 * page cache.
 */
#define READ_BUFFERS (2)
/* NEXT_BUFFER(): Loop over [0..(READ_BUFFERS-1)]. */
#define NEXT_BUFFER(_ix) ((_ix) ^ 1)

/*
 * BLKS_PER_BUF_SHIFT should be at least 2 to allow for "compressed"
 * data that takes up more space than the original and with unlucky
 * alignment.
 */
#define BLKS_PER_BUF_SHIFT	(2)
#define BLKS_PER_BUF		(1 << BLKS_PER_BUF_SHIFT)
#define BUFFER_SIZE		(BLKS_PER_BUF*PAGE_CACHE_SIZE)

static unsigned char read_buffers[READ_BUFFERS][BUFFER_SIZE];
static unsigned buffer_blocknr[READ_BUFFERS];
static struct super_block * buffer_dev[READ_BUFFERS];
static int next_buffer;

/*
 * Returns a pointer to a buffer containing at least LEN bytes of
 * filesystem starting at byte offset OFFSET into the filesystem.
 */
static void *cramfs_read(struct super_block *sb, unsigned int offset, unsigned int len)
{
	struct address_space *mapping = sb->s_bdev->bd_inode->i_mapping;
	struct page *pages[BLKS_PER_BUF];
	unsigned i, blocknr, buffer, unread;
	unsigned long devsize;
	char *data;

#if CRAMFS_BADBLOCK
	unsigned int raw_offset = offset;
	unsigned int xlat_blocknr = 0;

	// printk( "cramfs_read: input addr %08X, output addr %08X\n", offset, cramfs_bb_xlat(offset));
	offset = cramfs_bb_xlat(offset);  // do the bad block table translation
	
	//	if( (offset & ~0x3FFF) != ((offset + len) & ~0x3FFF) ) {
	//	  printk( "  Debug: Read at offset 0x%08lX length 0x%lX will fall over eraseblock boundary.\n", offset, len );
	//	}
#endif
	if (!len)
		return NULL;
	blocknr = offset >> PAGE_CACHE_SHIFT;  // (bunnie) this is 12 for ARM
	offset &= PAGE_CACHE_SIZE - 1;

	/* Check if an existing buffer already has the data.. */
	for (i = 0; i < READ_BUFFERS; i++) {
		unsigned int blk_offset;

		if (buffer_dev[i] != sb)
			continue;
		if (blocknr < buffer_blocknr[i])
			continue;
		blk_offset = (blocknr - buffer_blocknr[i]) << PAGE_CACHE_SHIFT;
		blk_offset += offset;
		if (blk_offset + len > BUFFER_SIZE)
			continue;
		return read_buffers[i] + blk_offset;
	}

	devsize = mapping->host->i_size >> PAGE_CACHE_SHIFT;

	/* Ok, read in BLKS_PER_BUF pages completely first. */
	unread = 0;
	for (i = 0; i < BLKS_PER_BUF; i++) {
		struct page *page = NULL;

#if CRAMFS_BADBLOCK
		xlat_blocknr = cramfs_bb_xlat(raw_offset + (i << PAGE_CACHE_SHIFT)) >> PAGE_CACHE_SHIFT;
		//		printk("  Debug: blocknr + i: %X, xlat_blocknr: %X\n", blocknr + i, xlat_blocknr);
		if (xlat_blocknr < devsize) {
		  page = read_cache_page(mapping, xlat_blocknr,
					 (filler_t *)mapping->a_ops->readpage,
					 NULL);
#else // original code
		if (blocknr + i < devsize) {
		  page = read_cache_page(mapping, blocknr + i,
					 (filler_t *)mapping->a_ops->readpage,
					 NULL);
#endif
			/* synchronous error? */
			if (IS_ERR(page))
				page = NULL;
		}
		pages[i] = page;
	}

	for (i = 0; i < BLKS_PER_BUF; i++) {
		struct page *page = pages[i];
		if (page) {
			wait_on_page_locked(page);
			if (!PageUptodate(page)) {
				/* asynchronous error */
				page_cache_release(page);
				pages[i] = NULL;
			}
		}
	}

	buffer = next_buffer;
	next_buffer = NEXT_BUFFER(buffer);
	buffer_blocknr[buffer] = blocknr;
	buffer_dev[buffer] = sb;

	data = read_buffers[buffer];
	for (i = 0; i < BLKS_PER_BUF; i++) {
		struct page *page = pages[i];
		if (page) {
			memcpy(data, kmap(page), PAGE_CACHE_SIZE);
			kunmap(page);
			page_cache_release(page);
		} else
			memset(data, 0, PAGE_CACHE_SIZE);
		data += PAGE_CACHE_SIZE;
	}
	return read_buffers[buffer] + offset;
}

#if CRAMFS_BADBLOCK
static void cramfs_init_badblocks(struct mtd_info *mtd) {
  int i;
  // int j; // delete when if 0 is permanently removed below
  int bbcount = 0;
  unsigned char pagedat[1024];
  int retlen;
  int addr = 0;
  unsigned int startOffset = 0;
  int inc = 0;

  // first thing to do, is to find where cramfs starts...
  // you can have bad blocks in other partitions pushing the absolute
  // address of this data "north"

  printk( "cramfs with NAND flash bad block extensions.\n" );
  printk( "Please remember to size your partitions with adequate margins for bad blocks.\n" );
  // read one page at the beginning of each erase block
  // because data can only be offset by a whole eraseblock
  for( addr = 0; addr < mtd->size; addr += mtd->erasesize ) {
    if( !mtd->block_isbad(mtd, addr) ) { // only consider good blocks!
      if( 0 == mtd->read(mtd, addr, mtd->oobblock * 2, &retlen, (u_char *) pagedat ) ) {
	if( *((unsigned long int *)pagedat) == CRAMFS_MAGIC ) {
	  printk( "cramfs_init_badblock: found start of cramfs at addr 0x%X\n", addr );
	  break;
	}
	if( *((unsigned long int *) &pagedat[512]) == CRAMFS_MAGIC ) {
	  printk( "cramfs_init_badblock: found start of cramfs at addr 0x%X\n", addr );
	  break;
	}
      } else {
	printk( "cramfs_init_badblocks: hit unreadable block during scan at 0x%X\n", addr );
	// for now, skip failed reads...could be a bad block, who knows.
      }
    }
  }
  // addr now has the residual "real" start of the cramfs filesystem

  if( addr == mtd->size ) { // check in the case that the user didn't put cramfs here...
    printk( "cramfs: fs magic number not found during badblock scan, filesystem is lost.\n" );
    // put some abort code here, but for now move on because we are debugging
    return;
  }

  startOffset = addr / mtd->erasesize; // because bad blocks in other partitions can cause this offset!
  //printk( "Debug: net startOffset is %06X with addr %08X and size %08X\n", startOffset, addr, mtd->erasesize );

  // then init the bad blocks to a default state (because the scan can "fall off the end")
  // this doesn't really solve the falling off the end problem but it at least makes the failure deterministic
  for( i = 0; i < bbTabMaxIndex; i++ ) {
    bbTable[i] = (startOffset + i) * mtd->erasesize;
  }

  // now scan for bad blocks
  // bad code, delete on final commit
#if 0
  inc = 1;
  for( i = 0; i < bbTabMaxIndex - bbcount; i++ ) { // subtract bbcount to avoid reading off the end of the partition
    if( (mtd->block_isbad( mtd, (i + bbcount + startOffset) * mtd->erasesize )) && (bbcount < bbTabMaxIndex) ) {
      printk( "cramfs_init_badblocks: block 0x%X is bad.\n", i );
      bbcount++; 
      // compensate for non-causal bad blocks
      // e.g., the blocks you didn't know about between you and your "real" good block
      // back when you originally calculated the offset
      for( j = 0; j < bbcount; j++ ) {
	bbTable[i-j] += mtd->erasesize;
      }
    }
    bbTable[i] = (startOffset + i + bbcount) * mtd->erasesize;
  }
#endif

  // much improved code
  addr = 0;
  bbcount = 0;
  for( i = 0; i < bbTabMaxIndex - bbcount; i++ ) { // subtract bbcount to avoid reading off the end of the partition
    if( (mtd->block_isbad( mtd, (i + startOffset) * mtd->erasesize )) && (bbcount < bbTabMaxIndex) ) {
      printk( "cramfs_init_badblocks: block 0x%X is bad.\n", i );
      inc = 0;
      bbcount++;
    } else {
      inc = 1; 
    }
    bbTable[addr] = (startOffset + i) * mtd->erasesize;
    if( inc ) 
      addr++;
  }

  //  printk( "Debug: bad block table for cramfs region:\n" );
  //  for( i = 0; i < bbTabMaxIndex; i++ ) {
  //    printk( "%04X: %04X\n", i, bbTable[i] / 0x4000 );
  //  }
}
#endif

static void cramfs_put_super(struct super_block *sb)
{
	kfree(sb->s_fs_info);
	sb->s_fs_info = NULL;
}

static int cramfs_remount(struct super_block *sb, int *flags, char *data)
{
	*flags |= MS_RDONLY;
	return 0;
}

static int cramfs_fill_super(struct super_block *sb, void *data, int silent)
{
	int i;
	struct cramfs_super super;
	unsigned long root_offset;
	struct cramfs_sb_info *sbi;
	struct inode *root;

#if CRAMFS_BADBLOCK
	int minor;
	struct mtd_info *mtd;                 // MTD driver pointer

	minor = MINOR(sb->s_dev);
	mtd = get_mtd_device(NULL, minor);
	bbTable = kmalloc((mtd->size / mtd->erasesize) * sizeof(unsigned int), GFP_KERNEL);
	bbTabMaxIndex = mtd->size / mtd->erasesize;
	bbBlockSize = mtd->erasesize; // we have to keep this around because we lose the mtd structure later on...
	if( NULL == bbTable ) 
	  printk( "Could not allocate bad block table for cramfs, expect a panic...\n" );

	cramfs_init_badblocks(mtd);
#endif

	sb->s_flags |= MS_RDONLY;

	sbi = kmalloc(sizeof(struct cramfs_sb_info), GFP_KERNEL);
	if (!sbi)
		return -ENOMEM;
	sb->s_fs_info = sbi;
	memset(sbi, 0, sizeof(struct cramfs_sb_info));

	/* Invalidate the read buffers on mount: think disk change.. */
	down(&read_mutex);
	for (i = 0; i < READ_BUFFERS; i++)
		buffer_blocknr[i] = -1;

	/* Read the first block and get the superblock from it */
	memcpy(&super, cramfs_read(sb, 0, sizeof(super)), sizeof(super));
	up(&read_mutex);

	/* Do sanity checks on the superblock */
	if (super.magic != CRAMFS_MAGIC) {
		/* check at 512 byte offset */
		down(&read_mutex);
		memcpy(&super, cramfs_read(sb, 512, sizeof(super)), sizeof(super));
		up(&read_mutex);
		if (super.magic != CRAMFS_MAGIC) {
			if (!silent)
				printk(KERN_ERR "cramfs: wrong magic\n");
			goto out;
		}
	}

	/* get feature flags first */
	if (super.flags & ~CRAMFS_SUPPORTED_FLAGS) {
		printk(KERN_ERR "cramfs: unsupported filesystem features\n");
		goto out;
	}

	/* Check that the root inode is in a sane state */
	if (!S_ISDIR(super.root.mode)) {
		printk(KERN_ERR "cramfs: root is not a directory\n");
		goto out;
	}
	root_offset = super.root.offset << 2;
	if (super.flags & CRAMFS_FLAG_FSID_VERSION_2) {
		sbi->size=super.size;
		sbi->blocks=super.fsid.blocks;
		sbi->files=super.fsid.files;
	} else {
		sbi->size=1<<28;
		sbi->blocks=0;
		sbi->files=0;
	}
	sbi->magic=super.magic;
	sbi->flags=super.flags;
	if (root_offset == 0)
		printk(KERN_INFO "cramfs: empty filesystem");
	else if (!(super.flags & CRAMFS_FLAG_SHIFTED_ROOT_OFFSET) &&
		 ((root_offset != sizeof(struct cramfs_super)) &&
		  (root_offset != 512 + sizeof(struct cramfs_super))))
	{
		printk(KERN_ERR "cramfs: bad root offset %lu\n", root_offset);
		goto out;
	}

	/* Set it all up.. */
	sb->s_op = &cramfs_ops;
	root = get_cramfs_inode(sb, &super.root);
	if (!root)
		goto out;
	sb->s_root = d_alloc_root(root);
	if (!sb->s_root) {
		iput(root);
		goto out;
	}
	return 0;
out:
	kfree(sbi);
	sb->s_fs_info = NULL;
	return -EINVAL;
}

static int cramfs_statfs(struct super_block *sb, struct kstatfs *buf)
{
	buf->f_type = CRAMFS_MAGIC;
	buf->f_bsize = PAGE_CACHE_SIZE;
	buf->f_blocks = CRAMFS_SB(sb)->blocks;
	buf->f_bfree = 0;
	buf->f_bavail = 0;
	buf->f_files = CRAMFS_SB(sb)->files;
	buf->f_ffree = 0;
	buf->f_namelen = CRAMFS_MAXPATHLEN;
	return 0;
}

/*
 * Read a cramfs directory entry.
 */
static int cramfs_readdir(struct file *filp, void *dirent, filldir_t filldir)
{
	struct inode *inode = filp->f_dentry->d_inode;
	struct super_block *sb = inode->i_sb;
	char *buf;
	unsigned int offset;
	int copied;

	/* Offset within the thing. */
	offset = filp->f_pos;
	if (offset >= inode->i_size)
		return 0;
	/* Directory entries are always 4-byte aligned */
	if (offset & 3)
		return -EINVAL;

	buf = kmalloc(256, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	copied = 0;
	while (offset < inode->i_size) {
		struct cramfs_inode *de;
		unsigned long nextoffset;
		char *name;
		ino_t ino;
		mode_t mode;
		int namelen, error;

		down(&read_mutex);
		de = cramfs_read(sb, OFFSET(inode) + offset, sizeof(*de)+256);
		name = (char *)(de+1);

		/*
		 * Namelengths on disk are shifted by two
		 * and the name padded out to 4-byte boundaries
		 * with zeroes.
		 */
		namelen = de->namelen << 2;
		memcpy(buf, name, namelen);
		ino = CRAMINO(de);
		mode = de->mode;
		up(&read_mutex);
		nextoffset = offset + sizeof(*de) + namelen;
		for (;;) {
			if (!namelen) {
				kfree(buf);
				return -EIO;
			}
			if (buf[namelen-1])
				break;
			namelen--;
		}
		error = filldir(dirent, buf, namelen, offset, ino, mode >> 12);
		if (error)
			break;

		offset = nextoffset;
		filp->f_pos = offset;
		copied++;
	}
	kfree(buf);
	return 0;
}

/*
 * Lookup and fill in the inode data..
 */
static struct dentry * cramfs_lookup(struct inode *dir, struct dentry *dentry, struct nameidata *nd)
{
	unsigned int offset = 0;
	int sorted;

	down(&read_mutex);
	sorted = CRAMFS_SB(dir->i_sb)->flags & CRAMFS_FLAG_SORTED_DIRS;
	while (offset < dir->i_size) {
		struct cramfs_inode *de;
		char *name;
		int namelen, retval;

		de = cramfs_read(dir->i_sb, OFFSET(dir) + offset, sizeof(*de)+256);
		name = (char *)(de+1);

		/* Try to take advantage of sorted directories */
		if (sorted && (dentry->d_name.name[0] < name[0]))
			break;

		namelen = de->namelen << 2;
		offset += sizeof(*de) + namelen;

		/* Quick check that the name is roughly the right length */
		if (((dentry->d_name.len + 3) & ~3) != namelen)
			continue;

		for (;;) {
			if (!namelen) {
				up(&read_mutex);
				return ERR_PTR(-EIO);
			}
			if (name[namelen-1])
				break;
			namelen--;
		}
		if (namelen != dentry->d_name.len)
			continue;
		retval = memcmp(dentry->d_name.name, name, namelen);
		if (retval > 0)
			continue;
		if (!retval) {
			struct cramfs_inode entry = *de;
			up(&read_mutex);
			d_add(dentry, get_cramfs_inode(dir->i_sb, &entry));
			return NULL;
		}
		/* else (retval < 0) */
		if (sorted)
			break;
	}
	up(&read_mutex);
	d_add(dentry, NULL);
	return NULL;
}

static int cramfs_readpage(struct file *file, struct page * page)
{
	struct inode *inode = page->mapping->host;
	u32 maxblock, bytes_filled;
	void *pgdata;

	maxblock = (inode->i_size + PAGE_CACHE_SIZE - 1) >> PAGE_CACHE_SHIFT;
	bytes_filled = 0;
	if (page->index < maxblock) {
		struct super_block *sb = inode->i_sb;
		u32 blkptr_offset = OFFSET(inode) + page->index*4;
		u32 start_offset, compr_len;

		start_offset = OFFSET(inode) + maxblock*4;
		down(&read_mutex);
		if (page->index)
			start_offset = *(u32 *) cramfs_read(sb, blkptr_offset-4, 4);
		compr_len = (*(u32 *) cramfs_read(sb, blkptr_offset, 4) - start_offset);
		up(&read_mutex);
		pgdata = kmap(page);
		if (compr_len == 0)
			; /* hole */
		else {
			down(&read_mutex);
			bytes_filled = cramfs_uncompress_block(pgdata,
				 PAGE_CACHE_SIZE,
				 cramfs_read(sb, start_offset, compr_len),
				 compr_len);
			up(&read_mutex);
		}
	} else
		pgdata = kmap(page);
	memset(pgdata + bytes_filled, 0, PAGE_CACHE_SIZE - bytes_filled);
	kunmap(page);
	flush_dcache_page(page);
	SetPageUptodate(page);
	unlock_page(page);
	return 0;
}

static struct address_space_operations cramfs_aops = {
	.readpage = cramfs_readpage
};

/*
 * Our operations:
 */

/*
 * A directory can only readdir
 */
static struct file_operations cramfs_directory_operations = {
	.llseek		= generic_file_llseek,
	.read		= generic_read_dir,
	.readdir	= cramfs_readdir,
};

static struct inode_operations cramfs_dir_inode_operations = {
	.lookup		= cramfs_lookup,
};

static struct super_operations cramfs_ops = {
	.put_super	= cramfs_put_super,
	.remount_fs	= cramfs_remount,
	.statfs		= cramfs_statfs,
};

static struct super_block *cramfs_get_sb(struct file_system_type *fs_type,
	int flags, const char *dev_name, void *data)
{
	return get_sb_bdev(fs_type, flags, dev_name, data, cramfs_fill_super);
}

static struct file_system_type cramfs_fs_type = {
	.owner		= THIS_MODULE,
	.name		= "cramfs",
	.get_sb		= cramfs_get_sb,
	.kill_sb	= kill_block_super,
	.fs_flags	= FS_REQUIRES_DEV,
};

static int __init init_cramfs_fs(void)
{
	cramfs_uncompress_init();
	return register_filesystem(&cramfs_fs_type);
}

static void __exit exit_cramfs_fs(void)
{
	cramfs_uncompress_exit();
	unregister_filesystem(&cramfs_fs_type);
}

module_init(init_cramfs_fs)
module_exit(exit_cramfs_fs)
MODULE_LICENSE("GPL");


/*
  bad block test case record (delete on final submission)

Bad eraseblock 236 at 0x003b0000
Bad eraseblock 240 at 0x003c0000
Bad eraseblock 244 at 0x003d0000
Bad eraseblock 248 at 0x003e0000
Bad eraseblock 249 at 0x003e4000
Bad eraseblock 251 at 0x003ec000
Bad eraseblock 2528 at 0x02780000


Block 0x000000EC is bad.
Block 0x000000F0 is bad.
Block 0x000000F4 is bad.
Block 0x000000F8 is bad.
Block 0x000000F9 is bad.
Block 0x000000FB is bad.
Block 0x000009E0 is bad.

240 X
241
242 <- startAddr=2   0  i = 0
243                  1  i = 1
244 X   bbcount = 1     i = 2
245                  2  i = 3
246                  3  i = 4
247                  4  i = 5
248 X   bbcount = 2     i = 6
249 X   bbcount = 3     i = 7
24A                  5  i = 8
24B X   bbcount = 4     i = 9
24C                  6  i = A
24D
24E

            236, 240
0000: 0002  242
0001: 0003  243
0002: 0005  245
0003: 0006  246
0004: 0008  247
0005: 000A  24A
0006: 000B  24C
0007: 000C  24D
0008: 000D  24E
0009: 000E
000A: 000F
*/
