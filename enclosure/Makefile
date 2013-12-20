all: enclosure_projection.svg

%.svg: %.dxf
	python dxf_input.py --auto=False $< > $@

%.pdf: %.svg
	inkscape -Cz --export-pdf=$@ --file=$<

%.dxf: %.scad
	openscad -o $@ $<

enclosure_projection.scad: make_enclosure.py
	python $<
