SRC=$(wildcard */*.scad)
PNG=$(patsubst %.scad,html/%.png,$(SRC))
HTML=$(patsubst %.scad,html/%.html,$(SRC))

OPENSCAD := openscad
ARGS := --imgsize=1200,900 --camera=0,0,0,55,0,25,180 --viewall

all : $(PNG) $(HTML) example-data.js assets

.PHONY: clean
clean :
	rm -rf html

.PHONY: assets
assets :
	cp -a assets html/

.PHONY: example-data.js
example-data.js :
	echo $(HTML)
	( \
	echo "openscad_examples = ["; \
	for a in Basics Functions Shapes Extrusion Advanced; \
	do \
		echo "  {"; \
		echo "    name : \"$$a\","; \
		echo "    files : ["; \
		for f in "$$a/"*.scad ; \
		do \
			echo "      \"`basename -s .scad $$f`\","; \
		done; \
		echo "    ]"; \
		echo "  },"; \
	done; \
	echo "];" \
	) > html/example-data.js

html/%.png : %.scad
	mkdir -p `dirname $@`
	$(OPENSCAD) $(ARGS) -o $@ $<

html/%.html : %.scad template-pre.html template-post.html
	echo $(notdir $(patsubst %.scad,%.png,$<))
	mkdir -p `dirname $@`
	sed -e 's/@@IMAGE@@/$(notdir $(patsubst %.scad,%.png,$<))/' template-pre.html > $@
	sed -e 's/</\&lt/; s/>/\&gt/;' $< >> $@
	cat template-post.html >> $@

#%.html : 
