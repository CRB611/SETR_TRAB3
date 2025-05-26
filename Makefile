CC      := gcc
CFLAGS := -Wall -Wextra -Wfatal-errors -ggdb -pthread \
          -DUNIT_TEST \
          -Istubs \
          -Iunity \
          -Imodules \
          -Itests


OBJDIR    := build
UNITY_OBJ := $(OBJDIR)/unity.o
UART_OBJ  := $(OBJDIR)/uart.o
RTDB_OBJ  := $(OBJDIR)/rtdb.o
PID_OBJ	  := $(OBJDIR)/pid.o
TESTS_OBJ := $(OBJDIR)/uart_tests.o $(OBJDIR)/rtdb_tests.o $(OBJDIR)/pid_tests.o
MAIN_OBJ  := $(OBJDIR)/testmain.o

TARGET := testmain

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(UNITY_OBJ) $(UART_OBJ) $(RTDB_OBJ) $(PID_OBJ) $(TESTS_OBJ) $(MAIN_OBJ)
	$(CC) $^ -o $@ $(CFLAGS) -lm


$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: unity/%.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/uart.o: modules/uart.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/rtdb.o: modules/rtdb.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/pid.o: modules/pid.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@	

$(OBJDIR)/uart_tests.o: tests/uart_tests.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/rtdb_tests.o: tests/rtdb_tests.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/pid_tests.o: tests/pid_tests.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@	

$(OBJDIR)/testmain.o: tests/testmain.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJDIR) *.o $(TARGET)

run: all
	./$(TARGET)
