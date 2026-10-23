.. SPDX-License-Identifier: GPL-2.0-only

=====================================================
Intel(R) Gaussian & Neural Accelerator (Intel(R) GNA)
=====================================================

Acronyms
--------
GNA	- Gaussian & Neural Accelerator
GMM	- Gaussian Mixer Model
CNN	- Convolutional Neural Network
RNN	- Recurrent Neural Networks
DNN	- Deep Neural Networks

Introduction
------------
The Intel(R) GNA is an internal PCI fixed device available on several Intel platforms/SoCs.
Feature set depends on the Intel chipset SKU.

Intel(R) GNA provides hardware accelerated computation for GMMs and Neural Networks.
It supports several layer types: affine, recurrent, and convolutional among others.
Hardware also provides helper layer types for copying and transposing matrices.

Linux Driver
------------
The driver also registers a DRM's render device to expose file operations via dev node.

The driver probes/removes a PCI device, implements file operations, handles runtime
power management, and interacts with hardware through MMIO registers.

Multiple processes can independently file many requests to the driver. These requests are
processed in a FIFO manner. The hardware can process one request at a time by using a FIFO
queue.

IOCTL
-----
Intel(R) GNA driver controls the device through IOCTL interfaces.
Following IOCTL commands - handled by DRM framework - are supported:

GNA_GET_PARAMETER gets driver and device capabilities.

GNA_GEM_NEW acquires new 4KB page aligned memory region ready for DMA operations.

GNA_GEM_FREE frees memory region back to system.

GNA_COMPUTE submits a request to the device queue.
            Memory regions acquired by GNA_GEM_NEW are part of request.

GNA_WAIT blocks and waits on the submitted request.

GNA MMU
-------
GNA’s MMU is being configured based on specific request memory usage. As the MMU can
address up to 256MB a single scoring request is limited to this amount of memory being
used.

GNA Library can allocate any number of memory regions for GNA usage. Its number and total
capacity are limited by the OSs’ resources. Due to GNA MMU restrictions, even when using
multiple memory regions, the sum of all the memory regions used within a single inference
request must be no larger than 256MB.

At least a single GNA memory region is needed to be allocated (and can be shared by
multiple models). At the other extreme, each GNA tensor (e.g.,
weights/biases/inputs/outputs) could use its own, separate GNA memory region.
