//! Slicing on non-contiguous data.

pub struct VecSlice<'a, T> {
    data:   &'a [T],
    length: uint,
    stride: uint
}

pub struct VecSliceMut<'a, T> {
    data:   &'a mut [T],
    length: uint,
    stride: uint
}

impl<'a, T> Collection for VecSlice<'a, T> {
    fn len(&self) -> uint {
        self.length
    }

    fn is_empty(&self) -> bool {
        self.length != 0
    }
}

impl<'a, T> Collection for VecSliceMut<'a, T> {
    fn len(&self) -> uint {
        self.length
    }

    fn is_empty(&self) -> bool {
        self.length != 0
    }
}

impl<'a, T> VecSlice<'a, T> {
    #[inline]
    pub fn new(data: &'a [T], length: uint, stride: uint) -> VecSlice<'a, T> {
        assert!(stride > 0, "The stride must at least be 1.");
        assert!(length == 0 || data.len() >= 1 + (length - 1) * stride, "The data buffer is too small.");

        VecSlice {
            data:   data,
            length: length,
            stride: stride
        }
    }

    #[inline(always)]
    fn id(&self, i: uint) -> uint {
        i * self.stride 
    }

    #[inline]
    pub fn get<'b>(&'b self, i: uint) -> &'b T {
        assert!(i < self.length);

        unsafe {
            self.unsafe_get(i)
        }
    }

    #[inline]
    pub unsafe fn unsafe_get<'b>(&'b self, i: uint) -> &'b T {
        self.data.unsafe_ref(self.id(i))
    }
}

impl<'a, T> VecSliceMut<'a, T> {
    #[inline]
    pub fn new(data: &'a mut [T], length: uint, stride: uint) -> VecSliceMut<'a, T> {
        assert!(stride > 0, "The stride must at least be 1.");
        assert!(length == 0 || data.len() >= 1 + (length - 1) * stride, "The data buffer is too small.");

        VecSliceMut {
            data:   data,
            length: length,
            stride: stride
        }
    }

    #[inline(always)]
    fn id(&self, i: uint) -> uint {
        i * self.stride 
    }

    #[inline]
    pub fn get<'b>(&'b self, i: uint) -> &'b T {
        assert!(i < self.length);

        unsafe {
            self.unsafe_get(i)
        }
    }

    #[inline]
    pub fn get_mut<'b>(&'b mut self, i: uint) -> &'b mut T {
        assert!(i < self.length);

        unsafe {
            self.unsafe_get_mut(i)
        }
    }

    #[inline]
    pub unsafe fn unsafe_get<'b>(&'b self, i: uint) -> &'b T {
        self.data.unsafe_ref(self.id(i))
    }

    #[inline]
    pub unsafe fn unsafe_get_mut<'b>(&'b mut self, i: uint) -> &'b mut T {
        let id = self.id(i);
        self.data.unsafe_mut_ref(id)
    }
}

impl<'a, T: Clone> VecSliceMut<'a, T> {
    #[inline]
    pub fn copy_from(&mut self, data: &VecSlice<T>) {
        assert!(data.len() == self.len());

        for i in range(0u, data.len()) {
            unsafe {
                *self.unsafe_get_mut(i) = data.unsafe_get(i).clone()
            }
        }
    }
}
