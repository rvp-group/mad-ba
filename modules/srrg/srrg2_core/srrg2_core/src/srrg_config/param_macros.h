#pragma once

#define PARAM(__type__, __name__, __description__, __value__, __flag__) \
  __type__ param_##__name__ = __type__(#__name__, __description__, this, __value__, __flag__)

#define PARAM_VECTOR(__type__, __name__, __description__, __flag__) \
  __type__ param_##__name__ = __type__(#__name__, __description__, this, __flag__)

#define PARAM_VECTOR_INIT(__type__, __name__, __description__, __value__, __flag__) \
  __type__ param_##__name__ = __type__(#__name__, __description__, this, __value__, __flag__)

#define PARAM_MH(__type__, __name__, __this__, __description__, __value__, __flag__) \
  __type__ param_##__name__ = __type__(#__name__, __description__, __this__, __value__, __flag__)

#define PARAM_VECTOR_MH(__type__, __name__, __description__, __this__, __flag__) \
  __type__ param_##__name__ = __type__(#__name__, __description__, __this__, __flag__)
