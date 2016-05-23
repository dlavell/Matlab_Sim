#!/bin/sh

pdflatex main.tex
#bibtex main
bibtex main
pdflatex main.tex
pdflatex main.tex
pdflatex main.tex

#make clean
mv main.pdf "Poster.pdf"
#evince "Poster.pdf" &
#zenity --error --text="Hell yeah! It compiled successfully\!" --title="Success!"


