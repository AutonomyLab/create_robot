uint8 song            # song number [0-3]
uint8 length          # song length [1-16]
uint8[] notes         # notes defined by the MIDI note numbering scheme. Notes outside the range of [31-127] are rest notes.
float32[] durations   # durations in seconds. Maximum duration is 255/64.
