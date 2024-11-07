extern crate alloc;
use core::usize;

use alloc::vec::Vec;
use axerrno::AxResult;

pub use arm_gicv2::GicInterface;
use axdevice_base::VCpuIf;

use log::*;
use spin::Mutex;

use crate::consts::*;
use crate::vgicc::Vgicc;
// use crate_interface::call_interface;
// pub use vcpu_if::*;

struct CoreState {
    id: u32,
    vmcr: u32,
    pending_lr: [u32; SPI_ID_MAX],
    saved_lr: [u32; GICH_LR_NUM],
    saved_elsr0: u32,
    saved_apr: u32,
    saved_hcr: u32,
    irq_no_mask: [u32; SPI_ID_MAX / 32],

    ppi_isenabler: u32,
    ppi_ipriorityr: [u8; PPI_ID_MAX],
}

struct VgicInner {
    used_irq: [u32; SPI_ID_MAX / 32],
    ptov: [u32; SPI_ID_MAX],
    vtop: [u32; SPI_ID_MAX],
    gicc: Vec<Vgicc>,

    ctrlr: u32,
    typer: u32,
    iidr: u32,
    real_pri: u32,

    core_state: Vec<CoreState>,

    gicd_igroupr: [u32; SPI_ID_MAX / 32],
    gicd_isenabler: [u32; SPI_ID_MAX / 32],
    gicd_ipriorityr: [u8; SPI_ID_MAX],
    gicd_itargetsr: [u8; SPI_ID_MAX],
    gicd_icfgr: [u32; SPI_ID_MAX / 16],
}

pub struct Vgic {
    inner: Mutex<VgicInner>,
    vcpu_num: usize,
}

impl Vgic {
    pub fn new() -> Vgic {
        let mut this = Self {
            inner: Mutex::new(VgicInner {
                used_irq: [0; SPI_ID_MAX / 32],
                ptov: [0; SPI_ID_MAX],
                vtop: [0; SPI_ID_MAX],
                gicc: Vec::new(),

                ctrlr: 0,
                typer: 0,
                iidr: 0,
                real_pri: 0,

                core_state: Vec::new(),

                gicd_igroupr: [0; SPI_ID_MAX / 32],
                gicd_isenabler: [0; SPI_ID_MAX / 32],
                gicd_ipriorityr: [0; SPI_ID_MAX],
                gicd_itargetsr: [0; SPI_ID_MAX],
                gicd_icfgr: [0; SPI_ID_MAX / 16],
            }),
            vcpu_num: 1,
        };
        Self::init(&mut this);
        this
    }

    fn init(this: &mut Self) {
        let mut vgic = this.inner.lock();
        for i in 0..SGI_ID_MAX {
            vgic.ptov[i] = i as u32;
            vgic.vtop[i] = i as u32;
        }

        for i in SGI_ID_MAX..SPI_ID_MAX {
            vgic.ptov[i] = u32::MAX;
            vgic.vtop[i] = u32::MAX;
        }

        for i in 0..SPI_ID_MAX / 32 {
            vgic.used_irq[i] = 0;
        }

        for i in 0..this.vcpu_num {
            vgic.core_state.push(CoreState {
                id: i as u32,
                vmcr: 0,
                pending_lr: [0; SPI_ID_MAX],
                saved_lr: [0; GICH_LR_NUM],
                saved_elsr0: 0,
                saved_apr: 0,
                saved_hcr: 0x5,
                irq_no_mask: [0; SPI_ID_MAX / 32],

                ppi_isenabler: 0xffff,
                ppi_ipriorityr: [0; PPI_ID_MAX],
            });
        }

        vgic.typer = GicInterface::get_typer();
        vgic.typer &= !(1 << 10);
        vgic.typer &= !0xE0;

        vgic.iidr = GicInterface::get_iidr();
        for i in 0..SPI_ID_MAX / 32 {
            vgic.gicd_igroupr[i] = 0;
        }

        vgic.gicd_icfgr[0] = 0xaaaa_aaaa;
        vgic.gicd_icfgr[1] = 0x5554_0000;
        for i in PPI_ID_MAX / 2..SPI_ID_MAX / 16 {
            vgic.gicd_icfgr[i] = 0x5555_5555;
        }

        vgic.real_pri = 0;
    }

    pub(crate) fn handle_read8(&self, addr: usize, vcpu: &dyn VCpuIf) -> AxResult<usize> {
        let value = self.handle_read32(addr, vcpu)?;
        return Ok((value >> (8 * (addr & 0x3))) & 0xff);
    }

    pub(crate) fn handle_read16(&self, addr: usize, vcpu: &dyn VCpuIf) -> AxResult<usize> {
        let value = self.handle_read32(addr, vcpu)?;
        return Ok((value >> (8 * (addr & 0x3))) & 0xffff);
    }

    pub(crate) fn handle_read32(&self, addr: usize, vcpu: &dyn VCpuIf) -> AxResult<usize> {
        match addr {
            VGICD_CTRL => Ok(self.inner.lock().ctrlr as usize),
            VGICD_TYPER => Ok(self.inner.lock().typer as usize),
            VGICD_IIDR => Ok(self.inner.lock().iidr as usize),
            VGICD_ISENABLER_X | VGICD_ICENABLER_X => {
                let id = vcpu.vcpu_id();
                Ok(self.inner.lock().core_state[id].ppi_isenabler as usize)
            }
            i if i >= VGICD_ISENABLER_X + 0x04 && i < VGICD_ICENABLER_X => {
                Ok(self.inner.lock().gicd_isenabler[(i - VGICD_ISENABLER_X) / 4] as usize)
            }
            i if i >= VGICD_ICENABLER_X + 0x04 && i < VGICD_ISPENDER_X => {
                Ok(self.inner.lock().gicd_isenabler[(i - VGICD_ICENABLER_X) / 4] as usize)
            }
            VGICD_ISPENDER_X => {
                let id = vcpu.vcpu_id();
                let mut value = self.inner.lock().core_state[id].irq_no_mask[0];
                for i in 0..GICH_LR_NUM {
                    let lr = VGICH_LR_X + i * 4;
                    if (lr & 1 << 28) != 0 && (lr & 0x1ff) / 32 == 0 {
                        value |= 1 << ((lr & 0x1ff) % 32)
                    }
                }
                return Ok(value as usize);
            }
            i if i >= VGICD_ISPENDER_X + 0x04
                && i < VGICD_ISPENDER_X + (SPI_ID_MAX / 32) * 0x04 =>
            {
                let mut value = 0;
                let idx = i - VGICD_ISPENDER_X / 4;
                for i in 0..self.vcpu_num {
                    value |= self.inner.lock().core_state[i].irq_no_mask[idx];
                    for j in 0..GICH_LR_NUM {
                        let lr = VGICH_LR_X + i * 4;
                        if (lr & 1 << 28) != 0 && (lr & 0x1ff) / 32 == idx {
                            todo!("Get LR for read");
                        }
                    }
                }
                return Ok(value as usize);
            }
            i if i >= VGICD_ICPENDER_X + 0x04
                && i < VGICD_ICPENDER_X + (SPI_ID_MAX / 32) * 0x04 =>
            {
                let mut value = 0;
                let idx = i - VGICD_ICPENDER_X / 4;
                for i in 0..self.vcpu_num {
                    value |= self.inner.lock().core_state[i].irq_no_mask[idx];
                    for j in 0..GICH_LR_NUM {
                        let lr = VGICH_LR_X + i * 4;
                        if (lr & 1 << 28) != 0 && (lr & 0x1ff) / 32 == idx {
                            todo!("Get LR for read");
                        }
                    }
                }
                return Ok(value as usize);
            }
            i if i >= VGICD_IPRIORITYR_X + (PPI_ID_MAX / 4) * 0x04
                && i <= VGICD_IPRIORITYR_X + (SPI_ID_MAX / 4) * 0x04 =>
            {
                let id = vcpu.vcpu_id();
                return Ok(self.inner.lock().core_state[id].ppi_ipriorityr
                    [i - VGICD_IPRIORITYR_X / 4] as usize);
            }
            i if i >= VGICD_ITARGETSR_X && i <= VGICD_ITARGETSR_X + (PPI_ID_MAX / 4) * 0x04 => {
                let id = vcpu.vcpu_id();
                let value = 1 << id;
                return Ok(value << 24 | value << 16 | value << 8 as usize);
            }
            i if i >= VGICD_ITARGETSR_X + (PPI_ID_MAX / 4) * 0x04
                && i < VGICD_ITARGETSR_X + (SPI_ID_MAX / 4) * 0x04 =>
            {
                return Ok(self.inner.lock().gicd_itargetsr[(i - VGICD_ITARGETSR_X) / 4] as usize)
            }
            i if i >= VGICD_ICFGR_X + PPI_ID_MAX / 16 * 0x04
                && i < VGICD_ICFGR_X + (SPI_ID_MAX / 16) * 0x04 =>
            {
                return Ok(self.inner.lock().gicd_icfgr[(i - VGICD_ICFGR_X) / 4] as usize);
            }
            _ => {
                error!("Unkonwn read addr: {:#x}", addr);
                Ok(0)
            }
        }
    }

    pub(crate) fn handle_write8(&self, addr: usize, val: usize, vcpu: &dyn VCpuIf) {
        match addr {
            // VGICD_CTLR => {
            //     error!("ctrl emu");
            //     // let curr_vcpu_id = call_interface!(VcpuIf::current_vcpu_id());
            //     // error!("current vcpu id: {}", curr_vcpu_id);

            //     // 这里只关心写入的最后两位，也就是 grp0 grp1
            //     let mut vgic_inner = self.inner.lock();
            //     vgic_inner.ctrlr = (val & 0b11) as u32;

            //     if vgic_inner.ctrlr > 0 {
            //         for i in SGI_ID_MAX..SPI_ID_MAX {
            //             if vgic_inner.used_irq[i / 32] & (1 << (i % 32)) != 0 {
            //                 GicInterface::set_enable(i, true);
            //                 // 设置优先级为0
            //                 GicInterface::set_priority(i, 0);
            //             }
            //         }
            //     } else {
            //         for i in SGI_ID_MAX..SPI_ID_MAX {
            //             if vgic_inner.used_irq[i / 32] & (1 << (i % 32)) != 0 {
            //                 GicInterface::set_enable(i, false);
            //             }
            //         }
            //     }
            //     // TODO: 告知其它PE开启或关闭相应中断
            // }
            // VGICD_ISENABLER_SGI_PPI..=VGICD_ISENABLER_SPI => {
            //     self.handle_write32(addr, val, vcpu);
            // }
            // VGICD_ISENABLER_SPI..=VGICD_ICENABLER_SGI_PPI => {}
            _ => {
                error!("Unkonwn write addr: {:#x}", addr);
            }
        }
    }

    pub(crate) fn handle_write16(&self, addr: usize, val: usize, vcpu: &dyn VCpuIf) {
        match addr {
            // VGICD_CTLR => self.handle_write8(addr, val, vcpu),
            // VGICD_ISENABLER_SGI_PPI..=VGICD_ISENABLER_SPI => {
            //     self.handle_write32(addr, val, vcpu);
            // }
            _ => {
                error!("Unkonwn write addr: {:#x}", addr);
            }
        }
    }

    pub(crate) fn handle_write32(&self, addr: usize, val: usize, vcpu: &dyn VCpuIf) {
        match addr {
            VGICD_CTRL => self.handle_write8(addr, val, vcpu),
            VGICD_ISENABLER_X => {
                let id = vcpu.vcpu_id();
                self.inner.lock().core_state[id].ppi_isenabler |= val as u32;

                for j in 0..32 {
                    if val & 1 << j != 0 {
                        let p = self.inner.lock().vtop[j];
                        if p >= 0 {
                            self.inner.lock().used_irq[p / 32] != 1 << (p % 32);
                        }
                    }
                }
            }
            // VGICD_ICENABLER_SGI_PPI => {}
            // VGICD_ISENABLER_SGI_PPI..=VGICD_ISENABLER_SPI => {
            //     error!("enabler emu");
            // }
            _ => {
                error!("Unkonwn write addr: {:#x}", addr);
            }
        }
    }
}
