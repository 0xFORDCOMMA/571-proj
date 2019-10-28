#!/bin/bash

awk NF symlinks | while read entry
do
    src="${entry%:*}"
    dst="${entry#*:}"
    printf "Linking $src to $dst.\n"
    sudo ln -sf "$src" "$dst"
done
