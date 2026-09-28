CXXFLAGS = -std=c++17 -Wall -Wextra -pedantic -O2

# Only main.cpp is compiled: it #includes src/*.cpp (the project uses no header files).
# Do NOT add src/*.cpp to the compile line, or every class would be defined twice.
traffic_sim: main.cpp $(wildcard src/*.cpp)
	$(CXX) $(CXXFLAGS) main.cpp -o traffic_sim

run: traffic_sim
	./traffic_sim

clean:
	rm -f traffic_sim

.PHONY: run clean
