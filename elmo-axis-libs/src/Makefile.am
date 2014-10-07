lib_LTLIBRARIES = libelmo-axis.la

libelmo_axis_la_SOURCES = \
	driveParam.c \
	harmonica.c \
	socketcan.c \
	socketcan.h \
	kemar.c \
	timeutils.c \
	timeutils.h

LIBS = $(ELMO_AXIS_LIBS)

AM_CPPFLAGS = -I$(top_srcdir)
AM_CFLAGS = -g -Wall $(ELMO_AXIS_CFLAGS)

libelmo_axisincludedir = $(includedir)/elmo-axis
libelmo_axisinclude_HEADERS = \
	$(top_srcdir)/elmo-axis/harmonica.h \
	$(top_srcdir)/elmo-axis/socketcan.h \
	$(top_srcdir)/elmo-axis/kemar.h
