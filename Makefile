CC = g++
TARGET = dlaf
COMPILE_FLAGS = -std=c++14 -flto -O3 -Wall -Wextra -pedantic -Wno-unused-parameter -march=native

all: $(TARGET)

$(TARGET): $(TARGET).cpp
	$(CC) $(COMPILE_FLAGS) -o $(TARGET) $(TARGET).cpp

clean:
	$(RM) $(TARGET)
