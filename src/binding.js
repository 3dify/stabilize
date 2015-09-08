var binding = require('../build/Release/binding');
module.exports = function(src, dest) {
  return binding.stabilize(src, dest);
};
