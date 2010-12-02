#
# Makefile for multifunction miscellaneous devices
#

obj-$(CONFIG_MCP)		+= mcp-core.o
obj-$(CONFIG_MCP_SA11X0)	+= mcp-sa11x0.o
obj-$(CONFIG_MCP_UCB1200)	+= ucb1x00-core.o
obj-$(CONFIG_MCP_UCB1200_TS)	+= ucb1x00-ts.o

ifeq ($(CONFIG_SA1100_ASSABET),y)
obj-$(CONFIG_MCP_UCB1200)	+= ucb1x00-assabet.o
endif

#obj-m := tsc2100.o
#tsc2100-objs := tsc2100-core.o tsc2100-ts.o

#obj-m := snd-imx21-tsc2100.o
#snd-imx21-tsc2100-objs	:= imx21-tsc2100.o
obj-m := chumby-tsc2100.o
