# Compiler and linker
CXX = g++
CXXFLAGS = -Wall -std=gnu++17

# Directories
SRC_DIR = .
OBJ_DIR = obj

# Files
SRC_FILES = $(SRC_DIR)/FonctionsImages.cpp $(SRC_DIR)/main.cpp $(SRC_DIR)/Image.cpp $(SRC_DIR)/image_ppm.cpp
OBJ_FILES = $(OBJ_DIR)/FonctionsImages.o $(OBJ_DIR)/main.o $(OBJ_DIR)/Image.o  $(OBJ_DIR)/image_ppm.o
EXEC = testFonction

# Targets
all: $(EXEC)

# Rule to build the executable
$(EXEC): $(OBJ_FILES)
	$(CXX) $(OBJ_FILES) -o $(EXEC)

# Rule to compile the source files to object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up object files and executable
clean:
	rm -rf $(OBJ_DIR) $(EXEC)

# Phony targets
.PHONY: all clean