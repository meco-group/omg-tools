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
PROBSOURCES = nlp.c hess_lag.c grad_f.c jac_g.c
PROBOBJECTS = $(PROBSOURCES:.c=.so)
SOURCES = $(SOURCEFILES) MotionPlanning.cpp
OBJECTS = $(SOURCES:.cpp=.o)
LIB = libomg.so
SRCDIR = src
OBJDIR = obj
OUTDIR = bin
OBJ = $(patsubst %,$(OBJDIR)/%,$(OBJECTS))
PROBOBJ = $(patsubst %,$(OUTDIR)/%,$(PROBOBJECTS))

all: dir $(OUTDIR)/$(EXECUTABLE)

lib: dir $(OUTDIR)/$(LIB)

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
