DEBUG_VALGRIND:=

valgrind: valgrind/Makefile
	$(MAKE) -C valgrind -j8 all && $(MAKE) $@-stage

valgrind/Makefile:
	$(MAKE) valgrind-configure

valgrind-configure:
	( cd valgrind ; \
		$(CONFIGURE) \
		--prefix=/usr \
		--bindir=/usr/sbin \
		--libdir=/usr/lib \
		CFLAGS="$(if $(DEBUG_VALGRIND),-g) $(EXTRACFLAGS)" \
		LDFLAGS="$(EXTRALDFLAGS)" \
	)

valgrind-install: valgrind
	install -D $(STAGEDIR)/usr/sbin/valgrind $(INSTALLDIR)/valgrind/usr/sbin/valgrind
	make -C valgrind -j8 DESTDIR=$(INSTALLDIR)/valgrind install
	@rm -fr $(INSTALLDIR)/valgrind/usr/{include,share}
	$(if $(DEBUG_VALGRIND),,$(STRIP) $(INSTALLDIR)/valgrind/usr/sbin/{cg_merge,valgrind,valgrind-di-server,valgrind-listener,vgdb})
	$(if $(DEBUG_VALGRIND),,$(STRIP) $(INSTALLDIR)/valgrind/usr/libexec/valgrind/{*-linux,*.so})

valgrind-clean:
	[ ! -f valgrind/Makefile ] || $(MAKE) -C valgrind clean
