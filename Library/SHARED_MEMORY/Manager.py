#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "February 29, 2024"
__project__   = "BRUCE"
__version__   = "0.0.5"
__status__    = "Product"

'''
Create a shared memory segment for all threads to use
'''

import mmap
import posix_ipc
import numpy as np


class SHMEMSEG(object):
    def __init__(self, robot_name='robot_name', seg_name='seg_name', init=False):
        """
        robot_name: name of robot
        seg_name:   root name of the memory segment
        init:       indicate whether this instance will create the segment, or just connect to it
        """
        self.robot_name = robot_name
        self.seg_name   = seg_name
        self.initialize = init
        self.blocks     = []
        self.mem_addr   = None
        self.mem_lock   = None
        self.mem_size   = 0

    def add_block(self, name, data):
        """
        Given name and data (numpy array), the block will be added in the segment.
        """
        block = {'name': name,
                 'data': data,
                 'size': data.size * data.itemsize,
                 'shape': data.shape,
                 'midx': self.mem_size}
        self.mem_size += block['size']
        self.blocks.append(block)

    def update_segment(self):
        """
        Parse the entire memory segment to a long array with column 1.
        """
        self.mem_data = np.empty((0, 1))  # need to reset every time
        for idx in range(len(self.blocks)):
            self.mem_data = np.concatenate((self.mem_data, self.blocks[idx]['data'].reshape(-1, 1)))

    def connect_segment(self):
        """
        Function that actually creates the memory block.
        """
        # Update the memory segment to get the total data size first!
        self.update_segment()

        # You can't mmap a size zero segment!
        if self.mem_size == 0:
            raise ValueError('You are trying to create an empty memory block! Please add blocks of memory!')

        # Path name to be used in /dev/shm
        path_name = self.robot_name + '_' + self.seg_name

        if self.initialize:
            try:
                # Close all shared memory if they already exist
                posix_ipc.unlink_shared_memory(path_name + '_mem')
                posix_ipc.unlink_semaphore(path_name + '_lock')
            except posix_ipc.ExistentialError:
                pass

            # Create handles for shared memory and semaphore
            mem = posix_ipc.SharedMemory(path_name + '_mem', posix_ipc.O_CREX, size=self.mem_size)
            self.mem_lock = posix_ipc.Semaphore(path_name + '_lock', posix_ipc.O_CREX)

            self.initialize = False
        else:
            # Open handles for shared memory and semaphore
            mem = posix_ipc.SharedMemory(path_name + '_mem')
            self.mem_lock = posix_ipc.Semaphore(path_name + '_lock')

        self.mem_addr = mmap.mmap(mem.fd, mem.size)
        self.mem_lock.release()
        mem.close_fd()  # close the file descriptor

    def set(self, val, opt='all'):
        """
        Set the data to the shared memory.
        :param val: {name: data}
        :param opt: 'all'  if setting all blocks
                    'only' if setting given blocks (so need to get the latest data from shared memory first)
        """
        if opt == 'only':
            with self.mem_lock:
                self.mem_addr.seek(0)
                mem_seg = np.ndarray(shape=(self.mem_size//self.mem_data.itemsize, 1), buffer=self.mem_addr)
                for idx in range(len(self.blocks)):
                    if self.blocks[idx]['name'] in val:
                        self.blocks[idx]['data'] = val[self.blocks[idx]['name']]
                    else:
                        idx0 = self.blocks[idx]['midx'] // self.blocks[idx]['data'].itemsize
                        idx1 = self.blocks[idx]['size'] // self.blocks[idx]['data'].itemsize + idx0
                        self.blocks[idx]['data'] = mem_seg[idx0:idx1].reshape(self.blocks[idx]['shape'])
                self.update_segment()
                self.mem_addr.seek(0)
                self.mem_addr.write(self.mem_data.data)
        elif opt == 'all':
            for idx in range(len(self.blocks)):
                if self.blocks[idx]['name'] in val:
                    self.blocks[idx]['data'] = val[self.blocks[idx]['name']]
            self.update_segment()
            with self.mem_lock:
                self.mem_addr.seek(0)
                self.mem_addr.write(self.mem_data.data)

    def get(self):
        """
        Get the data from the shared memory.
        """
        with self.mem_lock:
            self.mem_addr.seek(0)
            mem_seg = np.ndarray(shape=(self.mem_size//self.mem_data.itemsize, 1), buffer=self.mem_addr)

        data = {}
        for idx in range(len(self.blocks)):
            idx0 = self.blocks[idx]['midx'] // self.blocks[idx]['data'].itemsize
            idx1 = self.blocks[idx]['size'] // self.blocks[idx]['data'].itemsize + idx0
            self.blocks[idx]['data'] = mem_seg[idx0:idx1].reshape(self.blocks[idx]['shape'])
            data[self.blocks[idx]['name']] = self.blocks[idx]['data']

        return data
