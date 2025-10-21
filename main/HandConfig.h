// Copyright 2025 TetherIA, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// HandConfig.h
#pragma once

// Choose one via build flags OR by uncommenting a line below:
//   - PlatformIO:   build_flags = -DRIGHT_HAND   (or -DLEFT_HAND)
//   - Arduino CLI:  --build-property compiler.cpp.extra_flags="-DRIGHT_HAND"
//   - Arduino IDE:  just uncomment one of the lines here.

//#define LEFT_HAND
#define RIGHT_HAND

#if !defined(LEFT_HAND) && !defined(RIGHT_HAND)
  #error "Define exactly one: LEFT_HAND or RIGHT_HAND (build flag or uncomment in HandConfig.h)."
#endif

#if defined(LEFT_HAND) && defined(RIGHT_HAND)
  #error "Do not define both LEFT_HAND and RIGHT_HAND."
#endif
