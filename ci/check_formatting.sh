#!/bin/bash

find . -regex '.*\.\(cpp\|hpp\|c\|h\)' -exec clang-format -dry-run -Werror -style=file:".clang-format" -i {} +
formatting_violated=$?
if [ $formatting_violated -eq 1 ]; then
	exit 1
fi
