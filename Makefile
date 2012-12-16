CC := gcc
CFLAGS := -Wall -Wextra -std=gnu99
LDFLAGS := -pthread
BINDIR := bin
OBJDIR := obj
SRCDIR := src
DEPDIR := .depend
SOURCES := $(wildcard $(SRCDIR)/*.c)
OBJECTS := $(patsubst $(SRCDIR)/%.c,$(OBJDIR)/%.o, $(SOURCES))
DEPENDS := $(patsubst $(SRCDIR)/%.c,$(DEPDIR)/%.d, $(SOURCES))

$(BINDIR)/app: $(OBJECTS) | $(BINDIR)
	@echo "Linking $@"
	@$(CC) -o $@ $^ $(LDFLAGS)

-include $(DEPENDS)

$(OBJDIR)/%.o: $(SRCDIR)/%.c | $(OBJDIR)
	@echo "Compiling $*.c"
	@$(CC) $(CFLAGS) -c -o $@ $<

(DEPDIR)/%.d: $(SRCDIR)/%.c | $(DEPDIR)
	@$(CC) -MM -MG $< | sed 's!^\(.\+\).o:!$(DEPDIR)/\1.d $(OBJDIR)/\1.o:!' > $@

$(DEPDIR) $(BINDIR) $(OBJDIR):
	@mkdir $@

clean:
	@rm -rf $(BINDIR)/*
	@rm -rf $(OBJDIR)/*

.PHONY: clean

