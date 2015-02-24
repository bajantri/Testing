/* Synopsys QoS 4.00a device driver intregrated on CSR platform.
 *
 * Copyright (C) 2015 Vayavya Labs Pvt Ltd.
 *	http://www.vayavyalabs.com
 *
 * Author: Praveen Bajantri <praveen.bajantri@vayavyalabs.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "dwqos_avb.h"

/**
 * dwcqos_add_ubuff_entry - Add buffer entry in pdata
 * @pdata: driver private data pointer.
 * @ubuff: application buffer TODO: is this correct
 * Description:
 * TODO:
 */
static void dwcqos_add_ubuff_entry(struct dwcqos_prv_data *pdata,
					struct dwcqos_user_buff *ubuff)
{
	/*
	 * This is how all data bffers are stored in doubly link list
	 *
	 *	0[next, prev, ...]
	 *	   +	 |
	 *	   |	 +
	 *	1[next, prev, ...]
	 *	   +     |
	 *	   |	 +
	 *	2[next, prev, ...]
	 *	   +     |
	 *	   |	 +
	 *	3[next, prev, ...]
	 *
	 * ubuff->prev points to next node
	 * uuff->next points to previous node and
	 * pdata->uuff points to last/latest buffer allocated.
	 * */
	if (pdata->ubuff == NULL) {
		pdata->ubuff = ubuff;
	} else {
		ubuff->next = pdata->ubuff;
		pdata->ubuff->prev = ubuff;
		pdata->ubuff = ubuff;
	}
}

/**
 * dwcqos_del_ubuf_entry - Add buffer entry in pdata
 * @pdata: driver private data pointer.
 * @ubuff: application buffer TODO: is this correct
 * Description:
 * TODO:
 */
static void dwcqos_del_ubuf_entry(struct dwcqos_prv_data *pdata,
					struct dwcqos_user_buff *ubuff)
{
	if (ubuff->prev)
		ubuff->prev->next = ubuff->next;

	if (ubuff->next)
		ubuff->next->prev = ubuff->prev;

	if (ubuff == pdata->ubuff)
		pdata->ubuff = ubuff->next;

	vfree(ubuff);
}


/**
 * dwcqos_bind - Bind funtion to attach to a specific application
 * @file: File pointer(handler).
 * @argp: Argument pointer passed from application
 * Description:
 * This function does following,
 * a.	search for private data structure using req.pdev_name and
 *	updates valid pdata in file->private_data. This stored pdata
 *	is used later in the driver for hanlding other IOCTLs.
 * b.	gets the device CSR space and return it to user space through
 *	req.mmap_size.
 * Return value:
 * This function returns zero on success and -ve error number on failure
 * */
static int dwcqos_bind(struct file *file, void __user *argp)
{
	struct dwcqos_prv_data *pdata = NULL;
	struct dwcqos_bind_cmd req;
	struct resource *res = NULL;
	int ret = 0;

	if (copy_from_user(&req, argp, sizeof(req))) {
		pr_err("copy_from_user() failed\n");
		return -EFAULT;
	}

	/* get private data pointer */
	pdata = dwcqos_search_pdata(req.pdev_name);
	if (pdata == NULL) {
		pr_err("failed to get valid private data pointer\n");
		ret = -EFAULT;
		goto pdata_failed;
	}

	file->private_data = pdata;

	res = platform_get_resource(pdata->pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("Failed to get platform resource\n");
		return -ENODEV;
	}

	/* update device CSR space */
	req.mmap_size = ((mem->end) - (mem->start) + 1);

	if (copy_to_user(argp, &req, sizeof(req))) {
		pr_err("copy_to_user() failed\n");
		ret = -EFAULT;
		goto copy_failed;
	}

	return 0;

copy_failed:
	file->private_data = NULL;

pdata_failed:
	return ret;
}


/**
 * dwcqos_unbind - unbind funtion used by applicatiob
 * @file: File pointer(handler)
 * Description:
 * Release driver pdata structure.
 * Return value:
 * return zero on success and -ve error number on failure
 */
static int dwcqos_unbind(struct file *file)
{
	struct dwcqos_prv_data *pdata = file->private_data;

	if (pdata == NULL)
		return -EBADFD;

	file->private_data = NULL;

	return 0;
}


/**
 * dwcqos_get_link_param - Function to get current link parameters
 * @file: File pointer(handler).
 * @argp: Argument passed from application
 * Description:
 * This function is used by the application to get the current
 * link parameters
 * Return Value:
 * return zero on success and -ve error number on failure
 */
static int dwcqos_get_link_param(struct file *file, void __user *arg)
{
	struct dwcqos_prv_data *pdata = file->private_data;
	struct dwcqos_link_cmd req;

	if (pdata == NULL) {
		pr_err("map to unbound device\n");
		return -ENOENT;
	}

	/* get all link parameters from PHYLIB */
	req.up = pdata->phydev->link;
	req.speed = pdata->phydev->speed;
	req.duplex = pdata->phydev->duplex;

	if (copy_to_user(arg, &req, sizeof(req))) {
		pr_err("copy_to_user() failed\n");
		return -EFAULT;
	}

	return 0;
}

/**
 * dwcqos_map_tx_desc - Return the TX descriptor physical address and size
 * @pdata: private data pointer.
 * @req: pointer to proprietry structure used to pass information to driver
 * Description:
 * This function returns the TX descriptor physical address and size
 * based on cmd.
 * Return value:
 * return zero on success and -ve error number on failure.
 */
static long dwcqos_map_tx_desc(struct dwcqos_prv_data *pdata,
			      struct dwcqos_buf_cmd *req)
{
	int qInx = req->qInx;

	if (qInx > pdata->max_tx_queue_cnt) {
		pr_err("Invalid queue(%d) specified\n"
		       "Max number of Tx queue supported by device = %d\n",
		       qInx, pdata->max_tx_queue_cnt);
		return -EINVAL;
	}

	if (pdata->tx_avb_queue & (1 << qInx)) {
		pr_err("Tx descriptor memory for this queue(%d) is\
		       alreday in allocated\n", qInx);
		return -EBUSY;
	}

	//TODO: Indentation problem when used =
	/* allocate descriptor memory */
	GET_TX_DESC_PTR(qInx, 0) = dma_alloc_coherent(&(pdata->pdev->dev),
						      (sizeof(struct s_TX_NORMAL_DESC) * TX_DESC_CNT),
						      &(GET_TX_DESC_DMA_ADDR(qInx, 0)), GFP_KERNEL);

	if (GET_TX_DESC_PTR(qInx, 0) == NULL)
		return -ENOMEM;

	pdata->tx_avb_queue |= (1 << qInx);

	/* get required fields for user mode driver */
	req->phys_addr = GET_TX_DESC_DMA_ADDR(qInx, 0);
	req->mmap_size = (TX_DESC_CNT * sizeof(struct s_TX_NORMAL_DESC));

	return 0;
}

/**
 * dwcqos_map_rx_desc - Return the RX descriptor physical address and size
 * @pdata: private data pointer.
 * @req: pointer to proprietry structure used to pass information to driver
 * Description:
 * This function returns the RX descriptor physical address and size
 * based on cmd.
 * Return value:
 * return zero on success and -ve error number on failure.
 */
static long dwcqos_map_rx_desc(struct dwcqos_prv_data *pdata,
				struct dwcqos_buf_cmd *req)
{
	int qInx = req->qInx;

	if (qInx > pdata->max_rx_queue_cnt) {
		pr_err("Invalid queue(%d) specified\n"
			"Max number of Rx queue supported by device = %d\n",
			qInx, pdata->max_rx_queue_cnt);
		return -EINVAL;
	}

	if (pdata->rx_avb_queue & (1 << qInx)) {
		pr_err("Rx descriptor memory for this queue(%d) is\
			alreday in allocated\n", qInx);
		return -EBUSY;
	}

	//TODO: Indentation problem when used =
	/* allocate descriptor memory */
	GET_RX_DESC_PTR(qInx, 0) = dma_alloc_coherent(&(pdata->pdev->dev),
						      (sizeof(struct s_RX_NORMAL_DESC) * RX_DESC_CNT),
						      &(GET_RX_DESC_DMA_ADDR(qInx, 0)), GFP_KERNEL);
	if (GET_RX_DESC_PTR(qInx, 0) == NULL)
		return -ENOMEM;

	pdata->rx_avb_queue |= (1 << qInx);

	/* get required fields for user mode driver */
	req->phys_addr = GET_RX_DESC_DMA_ADDR(qInx, 0);
	req->mmap_size = (RX_DESC_CNT * sizeof(struct s_RX_NORMAL_DESC));

	return 0;
}


/**
 * dwcqos_map_desc - Return the descriptor physical address and size
 * @file: File pointer(handler).
 * @argp: Argument passed from application
 * @cmd: IOCTL command
 * Description:
 * This function returns the descriptor physical address and size based on cmd.
 * Return value:
 * return zero on success and -ve error number on failure.
 */
static long dwcqos_map_desc(struct file *file,
					void __user *arg,
					int cmd)
{
	struct dwcqos_prv_data *pdata = file->private_data;
	struct dwcqos_buf_cmd req;
	int ret = 0;


	if (pdata == NULL) {
		pr_err("map to unbound device\n");
		return -ENOENT;
	}

	if (copy_from_user(&req, arg, sizeof(req))) {
		pr_err("copy_from_user() failed\n");
		return -EFAULT;
	}

	if (cmd == DWC_ETH_QOS_MAP_TX_DESC)
		ret = dwcqos_map_tx_desc(pdata, &req);
	else
		ret = dwcqos_map_rx_desc(pdata, &req);

	if (copy_to_user(arg, &req, sizeof(req))) {
		pr_err("copy_to_user() failed\n");
		return -EFAULT;
	}

	return ret;
}

/**
 * dwcqos_unmap_tx_desc - Release the TX descriptor memory
 * @pdata: private data pointer.
 * @req: pointer to proprietry structure used to pass information to driver.
 * Description:
 * This function free's the allocated memory of TX descriptors
 * Return value:
 * return zero on success and -ve error number on failure.
 */
static int dwcqos_unmap_tx_desc(struct dwcqos_prv_data *pdata,
					struct dwcqos_buf_cmd *req)
{
	int qInx = req->qInx;

	if (qInx > pdata->max_tx_queue_cnt) {
		pr_err("Invalid queue(%d) specified\n"
			"Max number of tx queue supported by device = %d\n",
			qInx, pdata->max_tx_queue_cnt);
		return -EINVAL;
	}

	if ((pdata->tx_avb_queue & (1 << qInx)) == 0) {
		pr_err("Tx descriptor memory for this queue(%d) is\
			alreday in freed\n", qInx);
		return -EBUSY;
	}

	//TODO: correct the indentation
	/* free descriptor memory */
	if (GET_TX_DESC_PTR(qInx, 0)) {
		dma_free_coherent(&(pdata->pdev->dev),
				  (sizeof(struct s_TX_NORMAL_DESC) * TX_DESC_CNT),
				  GET_TX_DESC_PTR(qInx, 0),
				  GET_TX_DESC_DMA_ADDR(qInx, 0));
		GET_TX_DESC_PTR(qInx, 0) = NULL;
	}

	pdata->tx_avb_queue &= ~(1 << qInx);


	return 0;
}

/**
 * dwcqos_unmap_rx_desc - Release the RX descriptor memory
 * @pdata: private data pointer.
 * @req: pointer to proprietry structure used to pass information to driver.
 * Description:
 * This function free's the allocated memory of RX descriptors
 * Return value:
 * return zero on success and -ve error number on failure.
 */
static int dwcqos_unmap_rx_desc(struct dwcqos_prv_data *pdata,
					struct dwcqos_buf_cmd *req)
{
	int qInx = req->qInx;

	if (qInx > pdata->max_rx_queue_cnt) {
		pr_err("Invalid queue(%d) specified\n"
			"Max number of rx queue supported by device = %d\n",
			qInx, pdata->max_rx_queue_cnt);
		return -EINVAL;
	}

	if ((pdata->rx_avb_queue & (1 << qInx)) == 0) {
		pr_err("Rx descriptor memory for this queue(%d) is\
			alreday in freed\n", qInx);
		return -EBUSY;
	}

	/* free descriptor memory */
	if (GET_RX_DESC_PTR(qInx, 0)) {
		dma_free_coherent(&(pdata->pdev->dev),
				  (sizeof(struct s_RX_NORMAL_DESC) * RX_DESC_CNT),
				  GET_RX_DESC_PTR(qInx, 0),
				  GET_RX_DESC_DMA_ADDR(qInx, 0));
		GET_RX_DESC_PTR(qInx, 0) = NULL;
	}

	pdata->rx_avb_queue &= ~(1 << qInx);

	return 0;
}

/**
 * dwcqos_unmap_desc - Release the descriptor memory
 * @file: File pointer(handler).
 * @argp: Argument passed from application
 * @cmd: IOCTL command
 * Description:
 * This function free's the  allocated memory of TX/RX descriptors
 * Return value:
 * return zero on success and -ve error number on failure.
 */
static int dwcqos_unmap_desc(struct file *file,
				void __user *arg,
				int cmd)
{
	struct dwcqos_prv_data *pdata = file->private_data;
	struct dwcqos_buf_cmd req;
	int ret = 0;

	if (pdata == NULL) {
		pr_err("map to unbound device\n");
		return -ENOENT;
	}

	if (copy_from_user(&req, arg, sizeof(req))) {
		pr_err("copy_from_user() failed\n");
		return -EFAULT;
	}

	if (cmd == DWC_ETH_QOS_UNMAP_TX_DESC)
		ret = dwcqos_unmap_tx_desc(pdata, &req);
	else
		ret = dwcqos_unmap_rx_desc(pdata, &req);

	return ret;
}

/**
 * dwcqos_map_buf - Allocate memory for user mode buffer.
 * @file: File pointer(handler).
 * @arg: Argument passed from application
 * Description:
 * This API will allocate "arg->alloc_size" bytes of data
 * for application and fills other parameters of arg.
 * Return value:
 * return zero on success and -ve error number on failure.
 */
static int dwcqos_map_buf(struct file *file,
				void __user *arg)
{
	struct dwcqos_prv_data *pdata = file->private_data;
	struct dwcqos_user_buff *ubuff = NULL;
	struct dwcqos_buf_cmd req;
	void *addr = NULL;
	dma_addr_t dma_addr;
	int size = 0, ret = 0;

	if (pdata == NULL) {
		pr_err("map to unbound device\n");
		return -ENOENT;
	}

	if (copy_from_user(&req, arg, sizeof(req))) {
		pr_err("copy_from_user() failed\n");
		return -EFAULT;
	}

	ubuff = vzalloc(sizeof(struct dwcqos_user_buff));
	if (unlikely(!ubuff)) {
		ret = -ENOMEM;
		pr_err("failed to allocate user_buff\n");
		goto ubuff_err;
	}

	dwcqos_add_ubuff_entry(pdata, ubuff);

	/* get how much memory to be allocated */
	size = req.alloc_size;
	addr = dma_alloc_coherent((&pdata->pdev->dev), size,
				&dma_addr, GFP_KERNEL);
	if (unlikely(!addr)) {
		ret = -ENOMEM;
		pr_err("failed to allocate user data buffer\n");
		goto dma_err;
	}

	pdata->ubuff->addr = addr;
	pdata->ubuff->dma_addr = dma_addr;
	pdata->ubuff->size = size;

	req.phys_addr = dma_addr;
	req.mmap_size = size;

	if (copy_to_user(arg, &req, sizeof(req))) {
		ret = -EFAULT;
		pr_err("copy_to_user() failed\n");
		goto cpy_err;
	}

	return 0;

cpy_err:
	dma_free_coherent((&pdata->pdev->dev), size, addr, dma_addr);

dma_err:
	dwcqos_del_ubuf_entry(pdata, ubuff);

ubuff_err:
	return ret;
}

/**
 * dwcqos_unmap_buf - Free the memory allocated for user mode buffer
 * @file: File pointer(handler).
 * @arg: Argument passed from application
 * Description:
 * This function free's the memory allocated for user mode buffer
 * Return value:
 * return zero on success and -ve error number on failure.
 */
static int dwcqos_unmap_buf(struct file *file,
				void __user *arg)
{
	struct dwcqos_prv_data *pdata = file->private_data;
	struct dwcqos_user_buff *ubuff = NULL;
	struct dwcqos_buf_cmd req;
	int ret = 0;

	if (pdata == NULL) {
		pr_err("map to unbound device\n");
		return -ENOENT;
	}

	if (copy_from_user(&req, arg, sizeof(req))) {
		pr_err("copy_from_user() failed\n");
		return -EFAULT;
	}

	ubuff = pdata->ubuff;
	/* find the corresponding buffer and free */
	while (ubuff != NULL) {
		if (req.phys_addr == ubuff->dma_addr)
			break;
		ubuff = ubuff->next;
	}

	if (ubuff == NULL) {
		pr_err("already freed or not a valid buffer\n");
		return -EINVAL;
	}

	dma_free_coherent((&pdata->pdev->dev), ubuff->size,
			ubuff->addr, ubuff->dma_addr);

	dwcqos_dele_ubuf_entry(pdata, ubuff);

	return ret;
}

/**
 * dwcqos_rx_getlock - acquire semaphore for rx
 * @file: file pointer(handler).
 * @arg: argument passed from application
 * description:
 * TODO: write description
 * return value:
 * return zero on success and -ve error number on failure.
 */
static int dwcqos_rx_getlock(struct file *file, void __user *arg)
{
	struct dwcqos_prv_data *pdata = file->private_data;
	int sem_idx = 0, qInx;
	int ret = 0;

	if (pdata == NULL) {
		pr_err("map to unbound device\n");
		return -ENOENT;
	}

	if (copy_from_user(&qInx, arg, sizeof(qInx))) {
		pr_err("copy_from_user() failed\n");
		return -EFAULT;
	}

	sem_idx = (DWC_ETH_QOS_MAX_AVB_Q_CNT -
			(pdata->max_rx_queue_cnt - qInx));
	/* acquire semaphore */
	ret = down_interruptible(&pdata->rx_sem[sem_idx]);

	return ret;
}


/**
 * dwcqos_rx_setlock - release semaphore
 * @file: file pointer(handler).
 * @arg: argument passed from application
 * description:
 * TODO: write description
 * return value:
 * return zero on success and -ve error number on failure.
 */
static int dwcqos_rx_setlock(struct file *file, void __user *arg)
{
	struct dwcqos_prv_data *pdata = file->private_data;
	int sem_idx = 0, qInx;
	int ret = 0;

	if (pdata == NULL) {
		pr_err("map to unbound device\n");
		return -ENOENT;
	}

	if (copy_from_user(&qInx, arg, sizeof(qInx))) {
		pr_err("copy_from_user() failed\n");
		return -EFAULT;
	}
	sem_idx = (DWC_ETH_QOS_MAX_AVB_Q_CNT -
			(pdata->max_rx_queue_cnt - qInx));
	/* release semaphore */
	up(&pdata->rx_sem[sem_idx]);

	return ret;
}


static ssize_t dwcqos_read_file(struct file *file, char __user *buf,
			       size_t count, loff_t *pos)
{
	/* don't support reads for any status or data */
	return -EINVAL;
}


static ssize_t dwcqos_write_file(struct file *file, const char __user *buf,
					size_t count, loff_t *pos)
{
	/* don't support writes for any status or data */
	return -EINVAL;
}


static unsigned int dwcqos_poll_file(struct file *file,
					poll_table *wait)
{	/* don't support reads for any status or data */
	return -EINVAL;
}


static int dwcqos_open_file(struct inode *inode,
				struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/**
 * dwcqos_release_file - Return the descriptor physical address and size
 * @inode: TODO: what is inode ?
 * @file: file pointer(handler)
 * Description:
 * TODO:?
 * Return value:
 * return zero on success and -ve error number on failure.
 */
/* return zero on success and -ve error number on failure */
static int dwcqos_release_file(struct inode *inode,
					struct file *file)
{
	struct dwcqos_prv_data *pdata = file->private_data;
	struct dwcqos_user_buff *ubuff = NULL;
	int ret = 0;
	int qInx;

	if (pdata == NULL) {
		pr_err("map to unbound device\n");
		return 0;
	}

	/* free descriptor memory of all used queue */
	for (qInx = pdata->tx_avb_q_idx; qInx < pdata->tx_queue_cnt; qInx++) {
		if (pdata->tx_avb_queue & (1 << qInx)) {
			//TODO:indentation 
			if (GET_TX_DESC_PTR(qInx, 0)) {
				dma_free_coherent(&(pdata->pdev->dev),
						  (sizeof(struct s_TX_NORMAL_DESC) *
						   TX_DESC_CNT),
						  GET_TX_DESC_PTR(qInx, 0),
						  GET_TX_DESC_DMA_ADDR(qInx, 0));
				GET_TX_DESC_PTR(qInx, 0) = NULL;
			}
		}
		pdata->tx_avb_queue &= ~(1 << qInx);
	}

	ubuff = pdata->ubuff;
	/* free data buffers */
	while (ubuff != NULL) {
		dma_free_coherent((&pdata->pdev->dev), ubuff->size,
				  ubuff->addr, ubuff->dma_addr);
		dwcqos_del_ubuf_entry(pdata, ubuff);
		ubuff = pdata->ubuff;
	}

	ret = dwcqos_unbind(file);

	return ret;
}

static void dwcqos_vm_open(struct vm_area_struct *vma)
{
	/* nothing to do */
}


static void dwcqos_vm_close(struct vm_area_struct *vma)
{
	/* nothing to do */
}


static int dwcqos_vm_fault(struct vm_area_struct *area,
				struct vm_fault *fdata)
{
	/* nothing to do */
	return VM_FAULT_SIGBUS;
}


//TODO: Why are we having this ops when there is nothing to do
static struct vm_operations_struct dwcqos_mmap_ops = {
	.open = dwcqos_vm_open,
	.close = dwcqos_vm_close,
	.fault = dwcqos_vm_fault
};

/**
 * dwcqos_map_file - TODO
 * @file: File pointer(handler).
 * @vma: TODO
 * Description:
 * TODO:
 * Return value:
 * return zero on success and -ve error number on failure.
 */
static int dwcqos_mmap_file(struct file *file,
				struct vm_area_struct *vma)
{
	struct dwcqos_prv_data *pdata = file->private_data;
	unsigned long size = vma->vm_end - vma->vm_start;
	dma_addr_t pgoff = vma->vm_pgoff;
	dma_addr_t phys_addr;

	if (pdata == NULL) {
		pr_err("map to unbound device\n");
		return -ENODEV;
	}

	//TODO: change the code to platform..
	if (pgoff == 0)
		phys_addr = pci_resource_start(pdata->pdev, pdata->bar_no)
				>> PAGE_SHIFT;
	else
		phys_addr = pgoff;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, phys_addr,
			size, vma->vm_page_prot))
		return -EAGAIN;

	vma->vm_ops = &dwcqos_mmap_ops;

	return 0;
}


/**
 * dwcqos_ioctl - Entry point for the Ioctl
 * @file: File pointer(handler).
 * @cmd: IOCTL command
 * @arg: Argument used to pass the information to driver.
 * Description:
 * Currently it supports the phy_mii_ioctl(...) and HW time stamping.
 * Return Value:
 * 0 on success and an appropriate -ve integer on failure.
 */
static long dwcqos_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	long ret;

	switch (cmd) {
	case DWQOS_BIND:
		ret = dwcqos_bind(file, argp);
		break;
	case DWQOS_UNBIND:
		ret = dwcqos_unbind(file);
		break;
	case DWQOS_LINK_PARAM:
		ret = dwcqos_get_link_param(file, argp);
		break;
	case DWQOS_MAP_TX_DESC:
	case DWQOS_MAP_RX_DESC:
		ret = dwcqos_map_desc(file, argp, cmd);
		break;
	case DWQOS_UNMAP_TX_DESC:
	case DWQOS_UNMAP_RX_DESC:
		ret = dwcqos_unmap_desc(file, argp, cmd);
		break;
	case DWQOS_MAP_BUF:
		ret = dwcqos_map_buf(file, argp);
		break;
	case DWQOS_UNMAP_BUF:
		ret = dwcqos_unmap_buf(file, argp);
		break;
	case DWQOS_RX_GETLOCK:
		ret = dwcqos_rx_getlock(file, argp);
		break;
	case DWQOS_RX_TAKELOCK:
		ret = dwcqos_rx_setlock(file, argp);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static struct file_operations dwcqos_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = dwcqos_read_file,
	.write = dwcqos_write_file,
	.poll = dwcqos_poll_file,
	.open = dwcqos_open_file,
	.release = dwcqos_release_file,
	.mmap = dwcqos_mmap_file,
	.unlocked_ioctl = dwcqos_ioctl,
};

static struct miscdevice dwcqos_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "dwc_eth_qos_avb",
	.fops = &dwcqos_fops,
};

/**
 * dwcqos_misc_register - misc driver registration
 * Description:
 * misc driver registration for avb functionality.
 * Return value:
 * return zero on success and -ve error number on failure.
 */
int dwcqos_misc_register(void)
{
	return misc_register(&dwcqos_miscdev);
}

/**
 * dwcqos_misc_unregister - derregister a misc driver
 * Description:
 * misc driver deregistration of avb functionality.
 */
void dwcqos_misc_unregister(void)
{
	misc_deregister(&dwcqos_miscdev);
}
