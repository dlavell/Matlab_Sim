#compiler by Yegeta Zeleke
# remove clean_all if you prefer to keep the old pdf
make clean_all
make all
make clean
evince "Package_delivery.pdf" &
#zenity --error --text="Hell yeah! It compiled successfully\!" --title="Success!"

