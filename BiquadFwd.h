#pragma once

// Arduino IDE adds function prototypes for .ino files before user-defined types.
// Forward-declare here (and include this header from the .ino) so prototypes using Biquad compile.
struct Biquad;

