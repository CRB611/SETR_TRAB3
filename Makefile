# Makefile para compilar os testes Unity com stubs do Zephyr

CC        := gcc
CFLAGS    := -Wall -Wextra -Wfatal-errors -ggdb -pthread \
             -DUNIT_TEST \
             -Istubs \
             -Iunity \
             -Imodules \
             -Itests

# precisamos de -lm para pow()
LDLIBS    := -lm

OBJDIR      := build
MODULE_OBJS := $(OBJDIR)/uart.o
UNITY_OBJ   := $(OBJDIR)/unity.o
TESTS_OBJ   := $(OBJDIR)/uart_tests.o
MAIN_OBJ    := $(OBJDIR)/testmain.o

TARGET    := testmain

.PHONY: all clean run

all: $(TARGET)

# Link
$(TARGET): $(MODULE_OBJS) $(UNITY_OBJ) $(TESTS_OBJ) $(MAIN_OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

# compila o main do teste (ajusta o caminho conforme o teu)
$(MAIN_OBJ): tests/testmain.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

# compila uart.c usando o stub
$(MODULE_OBJS): modules/uart.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

# compila unity
$(UNITY_OBJ): unity/unity.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

# compila testes
$(TESTS_OBJ): tests/uart_tests.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

# garante que a pasta build existe
$(OBJDIR):
	mkdir -p $(OBJDIR)

clean:
	rm -rf $(OBJDIR)/*.o $(TARGET)

run: all
	./$(TARGET)
