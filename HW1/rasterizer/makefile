CXX         = g++
CXXFLAGS = -std=c++20
EXECUTABLE  = rasterizer.exe
OBJS   = image.o loader.o main.o rasterizer.o rasterizer_impl.o renderer.o
DATA   = sample-tests/task-triangle.yaml

####################################################################
# make sure to run 'make clean' first before changing between 'make debug' and 'make release'

release: CXXFLAGS += -O3 -DNDEBUG
release: $(EXECUTABLE)

debug: CXXFLAGS += -g3 -Wall -Wpedantic -DDEBUG -DPRINT_TRIG_DETAIL # -Werror
debug: $(EXECUTABLE)

$(EXECUTABLE): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $@

run: $(EXECUTABLE)
	./$(EXECUTABLE) $(DATA)

clean:
	rm -f $(OBJS)
	rm -f $(EXECUTABLE)

.PHONY: all release debug run clean

####################################################################
image.o: image.cpp 
loader.o: loader.cpp 
main.o: main.cpp 
rasterizer.o: rasterizer.cpp 
rasterizer_impl.o: rasterizer_impl.cpp 
renderer.o: renderer.cpp 