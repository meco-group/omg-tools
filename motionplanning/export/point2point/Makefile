CXX=g++
CC=gcc
CASADIDIR=@casadidir@
INCDIR=$(CASADIDIR)/include
LIBDIR=$(CASADIDIR)/casadi
CXXFLAGS=-I$(INCDIR) -DUSE_CXX11 -std=c++11 -fPIC -std=c++11
CFLAGS=-fPIC -shared -std=c99 @casadi_optiflags@
LDFLAGS=-L$(LIBDIR) -lcasadi -lstdc++ -lcasadi -Wl,-rpath $(LIBDIR)
PROBSOURCES=@casadi_src@
PROBOBJECTS=$(PROBSOURCES:.c=.so)
SOURCES=MotionPlanning.cpp main.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=MotionPlanning
SRCDIR = src
OBJDIR = obj
OUTDIR = bin
OBJ=$(patsubst %,$(OBJDIR)/%,$(OBJECTS))
PROBOBJ=$(patsubst %,$(OUTDIR)/%,$(PROBOBJECTS))

all: dir $(OUTDIR)/$(EXECUTABLE)

$(OUTDIR)/%.so: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS)

$(OUTDIR)/$(EXECUTABLE): $(OBJ) $(PROBOBJ)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LDFLAGS)

dir:
	mkdir -p $(OUTDIR)
	mkdir -p $(OBJDIR)

.PHONY: clean

clean:
	rm -rf $(OBJDIR) $(OUTDIR)
