//! Slicing on non-contiguous data.

/// A vector slice with a specific length and stride.
///
/// The stride is the number of index increments between two elements.
pub struct VecSlice<'a, T: 'a> {
    data:   &'a [T],
    length: uint,
    stride: uint
}

/// A mutable vector slice with a specific length and stride.
///
/// The stride is the number of index increments between two elements.
pub struct VecSliceMut<'a, T: 'a> {
    data:   &'a mut [T],
    length: uint,
    stride: uint
}

impl<'a, T> Collection for VecSlice<'a, T> {
    #[inline]
    fn len(&self) -> uint {
        self.length
    }

    #[inline]
    fn is_empty(&self) -> bool {
        self.length != 0
    }
}

impl<'a, T> Collection for VecSliceMut<'a, T> {
    #[inline]
    fn len(&self) -> uint {
        self.length
    }

    #[inline]
    fn is_empty(&self) -> bool {
        self.length != 0
    }
}

impl<'a, T> VecSlice<'a, T> {
    /// Creates a new immutable slice.
    #[inline]
    pub fn new(data: &'a [T], length: uint, stride: uint) -> VecSlice<'a, T> {
        assert!(stride > 0, "The stride must at least be 1.");
        assert!(length == 0 || data.len() >= 1 + (length - 1) * stride, "The data buffer is too small.");

        unsafe {
            VecSlice::new_unsafe(data, length, stride)
        }
    }

    /// Creates a new immutable slice. The size of the data buffer is not checked.
    #[inline]
    pub unsafe fn new_unsafe(data: &'a [T], length: uint, stride: uint) -> VecSlice<'a, T> {
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

    /// Gets the i-th element of the slice.
    ///
    /// This is the same as the `i * self.stride`-th element of the wrapped vector.
    #[inline]
    pub fn get<'b>(&'b self, i: uint) -> &'b T {
        assert!(i < self.length);

        unsafe {
            self.unsafe_get(i)
        }
    }

    /// Gets the i-th element of the slice without bound-checking.
    ///
    /// This is the same as the `i * self.stride`-th element of the wrapped vector.
    #[inline]
    pub unsafe fn unsafe_get<'b>(&'b self, i: uint) -> &'b T {
        self.data.unsafe_get(self.id(i))
    }
}

impl<'a, T> VecSliceMut<'a, T> {
    /// Creates a new mutable slice.
    #[inline]
    pub fn new(data: &'a mut [T], length: uint, stride: uint) -> VecSliceMut<'a, T> {
        assert!(stride > 0, "The stride must at least be 1.");
        assert!(length == 0 || data.len() >= 1 + (length - 1) * stride, "The data buffer is too small.");

        unsafe {
            VecSliceMut::new_unsafe(data, length, stride)
        }
    }

    /// Creates a new mutable slice. The size of the data buffer is not checked.
    #[inline]
    pub unsafe fn new_unsafe(data: &'a mut [T], length: uint, stride: uint) -> VecSliceMut<'a, T> {
        VecSliceMut {
            data:   data,
            length: length,
            stride: stride
        }
    }

    /// Creates an immutable slice from this mutable slice.
    #[inline]
    pub fn as_slice<'b>(&'b self) -> VecSlice<'b, T> {
        unsafe {
            VecSlice::new_unsafe(self.data, self.length, self.stride)
        }
    }

    #[inline(always)]
    fn id(&self, i: uint) -> uint {
        i * self.stride 
    }

    /// Gets the i-th element of the slice.
    ///
    /// This is the same as the `i * self.stride`-th element of the wrapped vector.
    #[inline]
    pub fn get<'b>(&'b self, i: uint) -> &'b T {
        assert!(i < self.length);

        unsafe {
            self.unsafe_get(i)
        }
    }

    /// Gets a mutable reference to the i-th element of the slice without bound-checking.
    ///
    /// This is the same as the `i * self.stride`-th element of the wrapped vector.
    #[inline]
    pub fn get_mut<'b>(&'b mut self, i: uint) -> &'b mut T {
        assert!(i < self.length);

        unsafe {
            self.unsafe_get_mut(i)
        }
    }

    /// Gets the i-th element of the slice without bound-checking.
    ///
    /// This is the same as the `i * self.stride`-th element of the wrapped vector.
    #[inline]
    pub unsafe fn unsafe_get<'b>(&'b self, i: uint) -> &'b T {
        self.data.unsafe_get(self.id(i))
    }

    /// Gets a mutable reference to the i-th element of the slice without bound-checking.
    ///
    /// This is the same as the `i * self.stride`-th element of the wrapped vector.
    #[inline]
    pub unsafe fn unsafe_get_mut<'b>(&'b mut self, i: uint) -> &'b mut T {
        let id = self.id(i);
        self.data.unsafe_mut_ref(id)
    }
}

impl<'a, T: Clone> VecSliceMut<'a, T> {
    /// Copy the content of another slice.
    /// Both slices must have the same length.
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
