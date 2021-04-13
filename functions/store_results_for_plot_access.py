# This file calls the results parser and stores the pre-processed data
# to be used in the plotting scripts.

import read_results

def main():
    print("GOT HERE YEE HAW")
    el = read_results.read_file("./results.txt")
    print(el)
    read_results.write_file(el)

if __name__ == "__main__":
    main()