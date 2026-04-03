CC      = gcc
CFLAGS  = -Wall -O2 -std=c99 -IC:/msys64/ucrt64/include/SDL2
LDFLAGS = -LC:/msys64/ucrt64/lib
LIBS    = -lmingw32 -lSDL2main -lSDL2 -lglew32 -lopengl32 -lm
TARGET  = globe.exe
SRCS    = main.c globe.c shader.c texture.c

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS) $(LIBS)

clean:
	rm -f $(TARGET)
