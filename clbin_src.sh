find ./ \( -path './build' -o -path './src/build' -o -path './.git' -o -path './.vscode/settings.json' -o -path './LICENSE' -o -path './clbin_src.sh' \) -prune -o -type f -exec printf "\n\n---------------------------File: {}--------------------------\n\n" ";" -exec cat {} ";" | curl -F 'clbin=<-' https://clbin.com
