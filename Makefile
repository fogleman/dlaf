CC = g++
TARGET = dlaf
COMPILE_FLAGS = -std=c++14 -DNDEBUG -flto -O3 -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-sign-compare -march=native -lboost_thread-mt

all: $(TARGET)

$(TARGET): $(TARGET).cpp
	$(CC) $(COMPILE_FLAGS) -o $(TARGET) $(TARGET).cpp

clean:
	$(RM) $(TARGET)
