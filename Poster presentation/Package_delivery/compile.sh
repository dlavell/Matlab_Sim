#!/bin/sh

pdflatex main.tex
#bibtex main
bibtex main
pdflatex main.tex
pdflatex main.tex
pdflatex main.tex

#make clean
mv main.pdf "Poster: QuadcopterTeam.pdf"
#evince "Poster: QuadcopterTeam.pdf" &
#zenity --error --text="Respect! It compiled successfully\!" --title="Success!"

