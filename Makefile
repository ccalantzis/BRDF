TARGET := brdf

LIBPATH := -L/usr/lib/xorg/modules/extensions
LIBS := `pkg-config --libs opencv` levmar/liblevmar.a -lstdc++ -lGL -lglut -lGLU -lopenblas

CXX := g++
LD  := g++

INCLUDE := -I/usr/local/include/opencv -I/usr/include
CXXFLAGS := -std=c++11 $(INCLUDE)
LDFLAGS := 

SRCS := brdfdata.cpp  glutcallbacks.cpp  main.cpp
OBJS := $(patsubst %.cpp,%.o,$(SRCS))

all: $(TARGET)

.PHONY: clean
clean:
	rm $(TARGET) $(OBJS)

$(TARGET): $(OBJS)
	echo $(OBJS)
	$(LD) $^ -o $@ $(LD_FLAGS) $(LIBPATH) $(LIBS)

