#!/bin/bash

print_usage()
{
	echo "Usage: $0 SRCDIR"
}

repeat() # CHAR COUNT
{
	if [ "$2" -gt 0 ]; then
		printf "$1"'%.s' $(eval "echo {1.."$(($2))"}")
	fi
}

gensrcs() # DIRECTORY INDENT
{
	name=$(basename "$1")

	if [ "$2" -gt 0 ]; then
		repeat '\t' $2; echo '$(addprefix '"$name/,\\"
	else
		echo "# Sources"
		echo "SRCDIR = $1"; echo; echo 'SRCS = $(addprefix $(SRCDIR)/,\'
	fi

	for file in "$1/"*; do
		name=$(basename "$file")
		if [[ -d "$file" ]]; then
			gensrcs "$file" $(($2 + 1))
		else
			repeat '\t' $(($2 + 1))
			echo "$name\\"
		fi
	done

	if [ "$2" -gt 0 ]; then
		repeat '\t' $2; echo ")\\"
	else
		echo ")"
	fi
}

# Abort on error
set -e

if [[ -z "$1" ]]; then
	print_usage
	exit 1
fi

gensrcs $1 0 > srcs.mk
