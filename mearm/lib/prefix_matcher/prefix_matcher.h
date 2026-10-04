#pragma once
// 匹配第一个完全匹配前缀的规则，并返回相应值
// Usage:
// PrefixRule<TypeOfValue> rules[] = {
//   {"test", val1,
//   {"t", val2,
//   ...
// };
//
// auto prefixMatcher = makePrefixMatcher(rules, fallbackVal);
//
// void loop() {
//   // ...
//   result = prefixMatcher.match(str);
// }
#include <stddef.h>

template <typename T>
struct PrefixRule {
  char const *prefix;
  T value;
};

template <size_t N, typename T>
class PrefixMatcher {
public:
  PrefixRule<T> const (&rules)[N];
  T fallback;

  T match(char const * const str) const;
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

template <size_t N, typename T>
PrefixMatcher<N, T> makePrefixMatcher(PrefixRule<T> const (&rules)[N], T fallback) {
  return PrefixMatcher<N, T>{rules, fallback};
}

template <size_t N, typename T>
T PrefixMatcher<N, T>::match(char const * const str) const {
  for (size_t i = 0; i < N; ++i) {
    if ( is_prefixMatch(rules[i].prefix, str) ) {
      return rules[i].value;
    }
  }
  return fallback;
}