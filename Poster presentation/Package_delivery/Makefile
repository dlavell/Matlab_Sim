#Senior design project
#by Yegeta,Daniel, Brandon

TEX = pdflatex -interaction nonstopmode
#BIB = bibtex
GS = pdflatex
 
PAPER = main
#uncomment if we have bib
#BIBFILE = main.bib

 
all: $(PAPER)
	$(TEX)$(PAPER).tex
#bib goes here	$(BIB)$(PAPER)
	$(TEX)$(PAPER).tex
	$(TEX)$(PAPER).tex

 
clean::
	rm -fv *.aux *.log *.blg *.toc *.out *.lot *.lof *.bbl *.out *.nav *.toc *.pdf

 
$(PAPER).pdf: $(PAPER).tex $(PAPER).bbl 
	$(TEX) $(PAPER) 
	$(TEX) $(PAPER)
 
$(PAPER).bbl: $(PAPER).tex $(BIBFILE)
	$(TEX) $(PAPER)
	$(BIB) $(PAPER)
