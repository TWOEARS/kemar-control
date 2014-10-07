bin_PROGRAMS = axisTest kemarTest

AM_CPPFLAGS = -I$(top_srcdir)
AM_CFLAGS = -g -Wall $(ELMO_AXIS_CFLAGS)

axisTest_SOURCES = axisTest.c
axisTest_LDADD = $(top_builddir)/src/libelmo-axis.la $(ELMO_AXIS_LIBS) \
		-lrt -lm -lpthread


kemarTest_SOURCES = kemarTest.c
kemarTest_LDADD = $(top_builddir)/src/libelmo-axis.la $(ELMO_AXIS_LIBS) \
		-lrt -lm -lpthread

