#ifdef GRH_DBUG_PRINTK

#include <linux/proc_fs.h>
#include <asm/uaccess.h>


typedef struct _dbug_page_ {
    struct _dbug_page_* next;
    int                 leng;
    char                data[ PAGE_SIZE - (sizeof(void*)+sizeof(int)) ];
} DBUG_PAGE;

static spinlock_t   dbug_lock;
static DBUG_PAGE*   dbug_head;
static DBUG_PAGE*   dbug_tail;
static int          dbug_pages = 0;
static int          dbug_max_pages = 1024;
static int          dbug_oneshot = 1;
static int          dbug_jiffies = 0;
int                 dbug_level = 0;
EXPORT_SYMBOL(dbug_level);


static void dbug_init( void )
{
	spin_lock_init(&dbug_lock);
    dbug_head  = NULL;
    dbug_tail  = NULL;
    dbug_pages = 0;
}


static void _free_first_page( void )
{
    DBUG_PAGE* page;

    page = dbug_head;
    if ( !page ) {
        dbug_pages = 0;
        return;
    }
    dbug_pages--;
    dbug_head = dbug_head->next;
    if ( dbug_head == NULL )
        dbug_tail = NULL;

    free_page( (unsigned long) page );
}


static void free_first_page( void )
{
    unsigned long   flags;

	spin_lock_irqsave(&dbug_lock, flags);
    _free_first_page();
	spin_unlock_irqrestore(&dbug_lock, flags);
}


static DBUG_PAGE*   append_new_page( void )
{
    DBUG_PAGE*    page;

    // if oneshot is enabled -- stop once all of the pages have been filled
    if (dbug_oneshot && dbug_pages >= dbug_max_pages) {
        return NULL;
    }
    page = ( DBUG_PAGE* ) get_zeroed_page( GFP_ATOMIC );

    // limit the size of the trace buffer based on dbug_max_pages setting
    while (dbug_pages >= dbug_max_pages && dbug_pages > 0) {
        _free_first_page();
    }

    if ( page )
        ++dbug_pages;
    else
    {
        // could not allocate another page, reuse the first page on the list
        // if possible
        if ( dbug_head == NULL )
            return NULL;
        if ( dbug_head == dbug_tail ) {
            memset( dbug_head, 0, PAGE_SIZE );
            return dbug_head;
        }
        page = dbug_head;
        dbug_head = dbug_head->next;
        if ( dbug_head == NULL )
            dbug_tail = NULL;

        memset( page, 0, PAGE_SIZE );
    }

    if ( dbug_head == NULL )
        dbug_head = page;
    else
        dbug_tail->next = page;
    dbug_tail = page;
    return page;
}


static void   append_new_data( char* text, int leng )
{
    unsigned long   flags;

	spin_lock_irqsave(&dbug_lock, flags);
    do {
        if ( dbug_tail && dbug_tail->leng + leng <= sizeof(dbug_tail->data) ) {
            memcpy( dbug_tail->data + dbug_tail->leng, text, leng );
            dbug_tail->leng += leng;
            break;
        }
        if (append_new_page() == NULL)
            break;
    } while ( dbug_tail != NULL );
	spin_unlock_irqrestore(&dbug_lock, flags);
}


int dbug_printk( const char* fmt, ... )
{
    char     text[ 256 ];
    int      leng;
    va_list  args;

    va_start(args, fmt);
    leng = vsnprintf( text, sizeof(text), fmt, args );
    va_end(args);
    append_new_data( text, leng );
    return leng;
}
EXPORT_SYMBOL(dbug_printk);


static struct proc_dir_entry* __init proc_file( 
         char* name, 
         struct proc_dir_entry* parent,
         int (*read_proc)( char* buf, char** start, off_t offset, 
                           int count, int* eof, void* data ),
         int (*write_proc)( struct file* file, const char* buf,
                            unsigned long count, void* data ),
         void* data )
{
    struct proc_dir_entry* entry;

    entry = create_proc_entry( name, 0, parent );
    if ( entry ) {
        entry->read_proc = read_proc;
        entry->write_proc = write_proc;
        entry->data = data;
    }

    return entry;
}


static int getval_from_user( struct file* file, 
                             const char* buf, 
                             u32 count, u32* value )
{
    int   len = count;
    char  string[32];

    if ( len > sizeof(string)-1 ) 
        len = sizeof(string)-1;
    if ( copy_from_user( string, buf, len ) )
        return -EFAULT;

    string[len] = '\0';
    *value = simple_strtoul( string, NULL, 0 );
    return 0;
}


static int proc_get_trace_cb( char* buf, char** start, off_t offset,
                              int count, int* eof, void* data )
{
    DBUG_PAGE*      page;
    unsigned long   flags;
    int             leng;
    off_t           off = offset;

	spin_lock_irqsave(&dbug_lock, flags);
    for ( page = dbug_head; page != NULL; page = page->next ) {
        if ( off < page->leng )
            break;
        off -= page->leng;
    }

    if ( page == NULL ) {
        *eof = 1;
        spin_unlock_irqrestore(&dbug_lock, flags);
        return 0;
    }

    leng = page->leng - off;
    if ( leng > count )
        leng = count;
    memcpy( buf, page->data + off, leng );
	spin_unlock_irqrestore(&dbug_lock, flags);
    *start = (char*)leng;
    return leng;
}


static int proc_set_trace_cb( struct file* file, const char* buf, 
                              unsigned long count, void* data )
{
    int             len = count;
    char            string[256];
    struct timeval  tv;

    if ( len > sizeof(string)-1 ) 
        len = sizeof(string)-1;
    if ( copy_from_user( string, buf, len ) )
        return -EFAULT;

    string[len] = '\0';
    if (dbug_jiffies)
        return dbug_printk("%lu-%s", jiffies, string);

    do_gettimeofday(&tv);
    return dbug_printk("%lu.%lu-%s", tv.tv_sec, tv.tv_usec, string);
}


static int proc_get_erase_cb( char* buf, char** start, off_t offset,
                              int count, int* eof, void* data )
{
    return 0;
}


static int proc_set_erase_cb( struct file* file, const char* buf, 
                              unsigned long count, void* data )
{
    while ( dbug_head ) {
        free_first_page();
    }
    return count;
}


static int proc_get_debugpages_cb( char* buf, char** start, off_t offset,
                                   int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d of %d\n", dbug_pages, dbug_max_pages );
}


static int proc_set_debugpages_cb( struct file* file, const char* buf, 
                                   unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc ) 
        return rc;

    dbug_max_pages = value;
    return count;
}


static int proc_get_debuglevel_cb( char* buf, char** start, off_t offset,
                                   int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", dbug_level );
}


static int proc_set_debuglevel_cb( struct file* file, const char* buf, 
                                   unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc ) 
        return rc;

    dbug_level = value;
    return count;
}


static int proc_get_oneshot_cb( char* buf, char** start, off_t offset,
                                int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", dbug_oneshot );
}


static int proc_set_oneshot_cb( struct file* file, const char* buf, 
                                unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc ) 
        return rc;

    dbug_oneshot = value;
    return count;
}


static int proc_get_jiffies_cb( char* buf, char** start, off_t offset,
                                int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", dbug_jiffies );
}


static int proc_set_jiffies_cb( struct file* file, const char* buf, 
                                unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc ) 
        return rc;

    dbug_jiffies = value;
    return count;
}


#define VOID_DIR       -1
#define ROOT_DIR        0
#define DBUG_DIR        1

static struct {
    char* name;
    int   parent;
    struct proc_dir_entry* entry;
} procdir[] = {
/*0*/    { "proc-root",     VOID_DIR, NULL },
/*1*/    { "chumby-dbug",   ROOT_DIR, NULL },
};


static struct {
    char* name;
    int   parent;
    int (*read_proc)( char* buf, char** start, off_t offset, 
                      int count, int* eof, void* data );
    int (*write_proc)( struct file* file, const char* buf,
                       unsigned long count, void* data );
    struct proc_dir_entry* entry;
} procfile[] = {
 { "level",  DBUG_DIR, proc_get_debuglevel_cb, proc_set_debuglevel_cb, NULL },
 { "pages",  DBUG_DIR, proc_get_debugpages_cb, proc_set_debugpages_cb, NULL },
 { "trace",  DBUG_DIR, proc_get_trace_cb,      proc_set_trace_cb,      NULL },
 { "erase",  DBUG_DIR, proc_get_erase_cb,      proc_set_erase_cb,      NULL },
 { "oneshot",DBUG_DIR, proc_get_oneshot_cb,    proc_set_oneshot_cb,    NULL },
 { "jiffies",DBUG_DIR, proc_get_jiffies_cb,    proc_set_jiffies_cb,    NULL },
};


static void dbug_proc_rmdir( void )
{
    int i;

    for ( i = ARRAY_SIZE(procdir)-1; i != 0; --i ) {
        if (procdir[i].entry) {
            remove_proc_entry( procdir[i].name, procdir[procdir[i].parent].entry );
            procdir[i].entry = NULL;
        }
    }
}


static void dbug_proc_exit( void )
{
	int i;

    for ( i = 0; i < ARRAY_SIZE(procfile); ++i ) {
        if (procfile[i].entry) {
            remove_proc_entry( procfile[i].name, procfile[procfile[i].parent].entry );
            procfile[i].entry = NULL;
        }
    }

    dbug_proc_rmdir();
}


static int dbug_proc_mkdir( void )
{
    int i;

    for ( i = 1; i < ARRAY_SIZE(procdir); ++i ) {
        procdir[i].entry = proc_mkdir(procdir[i].name, 
                                       procdir[procdir[i].parent].entry);
        if (procdir[i].entry == NULL) {
            dbug_proc_rmdir();
            return -1;
        }
    }

    return 0;
}


static int dbug_proc_init( void )
{
    int i;

    dbug_init();
    if ( dbug_proc_mkdir() )
        return -1;

    for ( i = 0; i < ARRAY_SIZE(procfile); ++i ) {
        struct proc_dir_entry* entry;

        entry = proc_file( procfile[i].name, procdir[procfile[i].parent].entry,
                           procfile[i].read_proc, procfile[i].write_proc, NULL );
        if ( entry == NULL ) {
            dbug_proc_exit();
            return -1;
        }

        procfile[i].entry = entry;
    }

    return 0;
}
#endif
