#pragma once
// 匹配第一个完全匹配前缀的规则，并执行相应函数
// Usage:
// LineBufferedInput input{Serial};
//
// PrefixRule rules[] = {
//   {"test", [](char const*const str){ /* ... */ }},
//   {"t", [](char const*const str){ /* ... */ }},
//   ...
// };
//
// auto prefixMatcher = makePrefixMatcher(rules, [](char const*const str){ /* ... */ });
//
// void loop() {
//   // ...
//   prefixMatcher.match(str);
// }
#include <stddef.h>

typedef void (*PrefixMatcherFunc)(char const*const);

struct PrefixRule {
  char const *prefix;
  PrefixMatcherFunc func; // IMPORTANT: 不得为nullptr
};

template <size_t N>
class PrefixMatcher {
public:
  PrefixRule (&rules)[N];
  PrefixMatcherFunc fallback;

  void match(char const * const str) const;
private:
  // Helper function
  static bool is_prefixMatch(char const * const prefix, char const * const str) {
    for (size_t i = 0; prefix[i] != '\0'; ++i) {
      if ( str[i] != prefix[i] )  return false;
    }
    return true;
  }
  // Helper function END
};

template <size_t N>
PrefixMatcher<N> makePrefixMatcher(PrefixRule (&rules)[N], PrefixMatcherFunc fallback) {
  return PrefixMatcher<N>{rules, fallback};
}

template <size_t N>
void PrefixMatcher<N>::match(char const * const str) const {
  for (size_t i = 0; i < N; ++i) {
    if ( is_prefixMatch(rules[i].prefix, str) ) {
      rules[i].func(str);
      return;
    }
  }
  fallback(str);
}