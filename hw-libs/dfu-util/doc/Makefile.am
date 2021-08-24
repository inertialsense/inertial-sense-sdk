man_MANS = dfu-util.1 dfu-suffix.1 dfu-prefix.1
EXTRA_DIST = .
dist-hook:
	rm $(distdir)/Makefile

%.1 : %.1.md
	pandoc $< -s -t man > $@
