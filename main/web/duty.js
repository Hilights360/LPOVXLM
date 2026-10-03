// Shared automatic duty controls on Controller and LEDs.
class DutyControls {
    constructor({post, refresh, notice}) {
        this.post=post;this.refresh=refresh;this.notice=notice;
        this.busy=false;this.wasAuto=false;this.wasPulse=false;this.status=null;
        this.box=document.getElementById('autoDuty');
        this.input=document.getElementById('duty');
        this.save=document.getElementById('saveDuty');
        this.info=document.getElementById('autodutyinfo');
        this.box.onchange=()=>this.toggle();
    }
    render(s,fill=false) {
        this.status=s;
        const enabled=!!s.settings.autoDuty,c=s.dutyControl||{},pulse=s.flashProof?.running;
        if(!this.busy)this.box.checked=enabled;
        this.box.disabled=this.busy||!!pulse;
        this.input.disabled=this.save.disabled=enabled||this.busy||!!pulse;
        if(pulse)this.input.value=Number(s.flashProof.nominalDutyPercent.toFixed(1));
        else if(enabled)this.input.value=c.available&&c.feasible?c.calculatedDuty:0;
        else if(fill||this.wasAuto||this.wasPulse)this.input.value=s.settings.duty;
        this.wasAuto=enabled;this.wasPulse=!!pulse;
        let text;
        if(pulse)text='Short flash verification is running: nominal '+s.flashProof.nominalPulse_us.toFixed(1)+' microseconds per flash. Saved duty and strobe resume when the test ends.';
        else if(!enabled)text='Manual duty. Check Auto-calc to follow RPM automatically.';
        else if(!c.available)text='Auto-calc is waiting for rotation and magnetic pulses. Rotation output stays dark.';
        else if(!c.feasible)text='Auto-calc: this RPM is too high for the current pixel and spoke counts. Rotation output stays dark. Reduce RPM or use fewer image spokes.';
        else if(c.continuous)text='Auto-calc: 100% continuous output at '+s.rpm.toFixed(1)+' RPM. There is no time for a dark gap. Reduce RPM or use fewer image spokes for narrower lines.';
        else text='Auto-calc: '+c.calculatedDuty+'% duty at '+s.rpm.toFixed(1)+' RPM ('+(s.fileSettings?.effectiveSpokes??s.settings.spokes)+' spokes). Updates as speed and transfer timing change.';
        if(enabled) {
            text+=' Saved manual duty: '+s.settings.duty+'%.';
            if(s.mode===14)text+=' The level pattern keeps its fixed widths.';
            else if(!c.active)text+=' Applies when playback or a rotation stability test runs.';
        }
        this.info.textContent=text;
        // Automatic duty takes precedence over the saved angular strobe.
        for(const id of ['strobe','strobeWidth']) {
            const element=document.getElementById(id);
            if(element)element.disabled=enabled||!!pulse;
        }
    }
    async toggle() {
        if(this.busy||!this.status)return;
        const enabled=this.box.checked;
        this.busy=true;this.render(this.status);
        try {
            await this.post('/autoduty',{enable:enabled?'1':'0'});
            await this.refresh();
            this.notice(enabled?'Auto-calc enabled.':'Auto-calc off. Manual timing restored.');
        } catch(error) { this.notice(error.message,true); }
        finally {this.busy=false;this.render(this.status);}
    }
}
