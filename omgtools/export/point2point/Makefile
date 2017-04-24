# This file is part of OMG-tools.
#
# OMG-tools -- Optimal Motion Generation-tools
# Copyright (C) 2016 Ruben Van Parys & Tim Mercy, KU Leuven.
# All rights reserved.
#
# OMG-tools is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA


#------------------------------------------------------------------------------
# Change this
#------------------------------------------------------------------------------
CASADILIB = @casadilib@
CASADIINC = @casadiinc@
CASADIOBJ = @casadiobj@
SOURCEFILES = @sourcefiles@
EXECUTABLE = @executable@
#------------------------------------------------------------------------------

CXX = g++
CC = gcc
INCDIR = $(CASADIINC)
LIBDIR = $(CASADILIB)
CXXFLAGS = -I$(INCDIR) -DUSE_CXX11 -std=c++11 -fPIC -std=c++11 -DCASADIOBJ="\"$(CASADIOBJ)\""
CFLAGS = -fPIC -shared -std=c99
LDFLAGS = -L$(LIBDIR) -lcasadi -lstdc++ -lcasadi -Wl,-rpath $(LIBDIR)
PROBSOURCES = @probsources@
PROBOBJECTS = $(PROBSOURCES:.c=.so)
SOURCES = $(SOURCEFILES) @sourcefiles2@
OBJECTS = $(SOURCES:.cpp=.o)
LIB = @libname@
SRCDIR = src
OBJDIR = obj
OUTDIR = bin
OBJ = $(patsubst %,$(OBJDIR)/%,$(OBJECTS))
PROBOBJ = $(patsubst %,$(OUTDIR)/%,$(PROBOBJECTS))

all: dir $(OUTDIR)/$(EXECUTABLE)

lib: dir $(OUTDIR)/$(LIB)

libdebug: DEBUG = -DDEBUG
libdebug: lib

debug: DEBUG = -DDEBUG
debug: all

$(OUTDIR)/%.so: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(DEBUG)

$(OUTDIR)/$(EXECUTABLE): $(OBJ) $(PROBOBJ)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(DEBUG) $(LDFLAGS)

$(OUTDIR)/$(LIB): $(OBJ) $(PROBOBJ)
	$(CXX) -o $@ $^ $(CXXFLAGS) -shared $(DEBUG) $(LDFLAGS)

dir:
	mkdir -p $(OUTDIR)
	mkdir -p $(OBJDIR)

.PHONY: clean

clean:
	rm -rf $(OBJDIR) $(OUTDIR)
