# Compiler and linker
CXX = g++
CXXFLAGS = -Wall -std=gnu++17 -O3 -march=native -flto -funroll-loops -fopenmp
LDFLAGS = -flto -fopenmp

# Directories
SRC_DIR = .
OBJ_DIR = obj

# Files
SRC_FILES = $(SRC_DIR)/FonctionsImages.cpp $(SRC_DIR)/main.cpp $(SRC_DIR)/Image.cpp $(SRC_DIR)/image_ppm.cpp
OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRC_FILES))
EXEC = testFosi nction

# Targets
all: $(EXEC)

# Rule to build the executable
$(EXEC): $(OBJ_FILES)
	$(CXX) $(CXXFLAGS) $(OBJ_FILES) -o $(EXEC) $(LDFLAGS)

# Rule to compile the source files to object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Profiling mode
profile: CXXFLAGS += -fprofile-generate
profile: clean all
	./$(EXEC)  # Exécute le programme pour générer les profils
	mv *.gcda *.gcno $(OBJ_DIR) 2>/dev/null || true

optimize: CXXFLAGS += -fprofile-use
optimize: clean all

# Clean up object files, executable, and profiling data
clean:
	rm -rf $(OBJ_DIR) $(EXEC) *.gcda *.gcno

# Phony targets
.PHONY: all clean profile optimize
