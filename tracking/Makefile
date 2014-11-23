CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`

track : track.cpp
	g++ $(CFLAGS) $(LIBS) -o $@ $<
