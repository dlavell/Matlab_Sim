#Package Delivery using drones
#By Yegeta Zeleke
# please refer to the readme file for documentation

TEX = pdflatex
#BIB = bibtex
GS = pdflatex
 
PAPER = Package_delivery
#uncomment if we have bib
#BIBFILE = rough.bib

#pdflatex Package_delivery.tex
 
all:$(PAPER).pdf
#clean after creating a pdf file remove unnecessary files
.PHONY: clean
clean::
	rm -fv *.aux *.log *.blg *.toc *.out *.lot *.lof *.bbl

clean_all::
	rm -fv *.aux *.log *.blg *.toc *.out *.lot *.lof *.bbl *.pdf
 
#added depency with the reference
$(PAPER).pdf: $(PAPER).tex $(PAPER).bbl 
	$(TEX) $(PAPER) 
	$(TEX) $(PAPER)
$(PAPER).bbl: $(PAPER).tex $(BIBFILE)
	$(TEX) $(PAPER)
	$(BIB) $(PAPER)

