## stabilize set of images

### usage

install with node-gyp

```bash
  $ git clone git@github.com:3Dify/stabilize.git
  $ cd myProject
  $ npm install ../stabilize # <= this should build the module
```

usage

```javascript
var stabilize = require('stabilize');

// existing images array
var orig = ['img1.jpg', 'img2.jpg'];
// non existing image names with same length
var dest = ['img1_d.jpg', 'img2_d.jpg'];

stabilize(orig, dest);
```

### notes

needs opencv to be installed in the system and the normal npm build tools

it will chash irrecuperabily if there are not enough matches between two contiguous
images to calculate the displacement

the execution will completelly block node until finishing, all is very untested atm

the uderlying algorithm can be found here http://nghiaho.com/uploads/videostabKalman.cpp


