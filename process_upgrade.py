
# -*- coding: utf-8 -*-
"""
Process Monitor (Basic) — Tkinter only
- 생산 공정(오븐 온도 T, 라인 속도 v, 습도 H)의 모니터링
- 결함률(%)을 간단 모델로 추정, 임계치 경고
- 시작/정지, 주기(ms), CSV 로깅
- 표준 라이브러리만 사용
"""
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import random, csv, time
count = 0
PAD = 6

def clamp(x, a, b): return max(a, min(b, x))

# 단순 공정 모델(교육용)
# 최적: T≈180°C, v≈40개/분, H≈45%
def defect_rate_pct(T, v, H):
    # 온도/속도는 이차 패널티, 습도는 55% 이상에서 선형 패널티
    pT = ((T - 180.0) / 15.0)**2
    pv = ((v - 40.0) / 10.0)**2
    pH = max(0.0, (H - 55.0) / 15.0)
    base = 1.5
    rate = base + 7.0*pT + 5.0*pv + 4.0*pH
    return float(clamp(rate, 0.0, 100.0))

class CsvLogger:
    def __init__(self):
        self.f = None; self.w = None
    def open(self, path):
        self.close()
        self.f = open(path, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow(["timestamp","T_C","v_ipm","H_pct","defect_pct","yield_pct","throughput_ipm","alarm"])
    def write(self, row):
        if self.w: self.w.writerow(row)
    def close(self):
        if self.f:
            self.f.close(); self.f=None; self.w=None

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Process Monitor (upgrade)")
        self.geometry("780x440")

        # 입력 파라미터(설정치)
        self.T_set = tk.StringVar(value="180")   # °C
        self.v_set = tk.StringVar(value="40")    # items/min
        self.H_env = tk.StringVar(value="45")    # %
        self.period = tk.StringVar(value="500")  # ms
        self.def_thr = tk.StringVar(value="5.0") # 결함률 임계치 %

        # GUI 레이아웃
        top = ttk.LabelFrame(self, text="설정")
        top.pack(fill="x", padx=PAD, pady=PAD)
        for i,(name,var,w) in enumerate([("오븐 온도 T [°C]", self.T_set, 8),
                                         ("라인 속도 v [개/분]", self.v_set, 8),
                                         ("환경 습도 H [%]", self.H_env, 8),
                                         ("업데이트 주기 [ms]", self.period, 8),
                                         ("결함률 임계치 [%]", self.def_thr, 8)]):
            ttk.Label(top, text=name).grid(row=i//3, column=(i%3)*2, sticky="e", padx=PAD, pady=PAD)
            ttk.Entry(top, textvariable=var, width=w).grid(row=i//3, column=(i%3)*2+1, padx=(0, PAD), pady=PAD)
        
        optf = ttk.LabelFrame(self, text="최적화(격자탐색)")
        optf.pack(fill="x", padx=PAD, pady=PAD)
        self.Tmin_var = tk.StringVar(value="140")
        self.Tmax_var = tk.StringVar(value="220")
        self.Tstep_var = tk.StringVar(value="2")
        self.vmin_var = tk.StringVar(value="20")
        self.vmax_var = tk.StringVar(value="60")
        self.vstep_var = tk.StringVar(value="1")
        self.vtarget_var = tk.StringVar(value="38")
        for i, (txt, var,w) in enumerate([("T_min", self.Tmin_var,8),
                                        ("T_max", self.Tmax_var,8),
                                        ("T_step", self.Tstep_var,8)]):
            ttk.Label(optf, text=txt).grid(row=0, column=i*2, sticky="e", padx=3)
            ttk.Entry(optf, textvariable=var, width=w).grid(row=0, column=i*2+1, padx=(0, PAD))
        for i, (txt, var,w) in enumerate([("v_min", self.vmin_var,8),
                                        ("v_max", self.vmax_var,8),
                                        ("v_step", self.vstep_var,8),
                                        ("목표 처리량 v_target", self.vtarget_var,8)]):
            ttk.Label(optf, text=txt).grid(row=1, column=i*2, sticky="e", padx=3)
            ttk.Entry(optf, textvariable=var, width=w).grid(row=1, column=i*2+1, padx=(0, PAD))



        ttk.Button(optf, text="최적화 실행",command=self.optimization).grid(row=2, column=0, columnspan=8,sticky="we", padx=PAD, pady=(PAD,0))

        ctrl = ttk.Frame(self); ctrl.pack(fill="x", padx=PAD, pady=(0, PAD))
        self.btn_start = ttk.Button(ctrl, text="시작", command=self.start)
        self.btn_stop  = ttk.Button(ctrl, text="정지", command=self.stop, state="disabled")
        self.btn_start.pack(side="left"); self.btn_stop.pack(side="left", padx=(PAD,0))
        self.log_enabled = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctrl, text="CSV 로깅", variable=self.log_enabled).pack(side="left", padx=(10,0))
        ttk.Button(ctrl, text="CSV 파일 선택", command=self.choose_csv).pack(side="left", padx=(PAD,0))
        ttk.Button(ctrl, text="초기화", command=self.reset).pack(side="right")

        # 측정 표시
        meter = ttk.LabelFrame(self, text="측정/계산 값")
        meter.pack(fill="x", padx=PAD, pady=PAD)
        self.T_meas_var = tk.StringVar(value="T: -- °C")
        self.v_meas_var = tk.StringVar(value="v: -- 개/분")
        self.H_var      = tk.StringVar(value="H: -- %")
        self.def_var    = tk.StringVar(value="결함률: -- %")
        self.yield_var  = tk.StringVar(value="수율: -- %")
        self.thr_var    = tk.StringVar(value="임계치: --")
        for i,var in enumerate([self.T_meas_var, self.v_meas_var, self.H_var, self.def_var, self.yield_var, self.thr_var]):
            ttk.Label(meter, textvariable=var, width=20).grid(row=0, column=i, padx=4, pady=4)

        # 상태 표시
        self.status_var = tk.StringVar(value="상태: 대기")
        ttk.Label(self, textvariable=self.status_var, anchor="w").pack(fill="x", padx=PAD, pady=(0, PAD))

        # 내부 상태
        self.running = False
        self.after_id = None
        self.logger = CsvLogger()
        self.csv_path = None
        # 시뮬레이터 내부 베이스
        self._T_base = 180.0
        self._v_base = 40.0

    def choose_csv(self):
        path = filedialog.asksaveasfilename(
            title="CSV 로그 파일 선택",
            defaultextension=".csv",
            filetypes=[("CSV files","*.csv"),("All files","*.*")],
            initialfile="process_log.csv"
        )
        if path:
            try:
                self.logger.open(path)
                self.csv_path = path
                self.status_var.set(f"CSV: {path}")
            except Exception as e:
                messagebox.showerror("오류", f"CSV 열기 실패: {e}")

    def reset(self):
        self.status_var.set("상태: 초기화")
        self.T_meas_var.set("T: -- °C")
        self.v_meas_var.set("v: -- 개/분")
        self.H_var.set("H: -- %")
        self.def_var.set("결함률: -- %")
        self.yield_var.set("수율: -- %")
        self.thr_var.set("임계치: --")

    def start(self):
        if self.running: return
        self.running = True
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.loop()

    def stop(self):
        self.running = False
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")
        if self.after_id:
            self.after_cancel(self.after_id); self.after_id=None
        try: self.logger.close()
        except Exception: pass
    

    def _read_inputs(self):
        # 잘못된 입력은 기본으로 대체
        try: T = float(self.T_set.get())
        except: T = 180.0; self.T_set.set("180")
        try: v = float(self.v_set.get())
        except: v = 40.0; self.v_set.set("40")
        try: H = float(self.H_env.get())
        except: H = 45.0; self.H_env.set("45")
        try: per = max(100, int(self.period.get()))
        except: per = 500; self.period.set("500")
        try: thr = float(self.def_thr.get())
        except: thr = 5.0; self.def_thr.set("5.0")
        return T, v, H, per, thr

    def _simulate_meas(self, T_set, v_set):
        # 천천한 드리프트 + 소량 노이즈
        self._T_base += 0.05*(random.random()-0.5)
        self._v_base += 0.1*(random.random()-0.5)
        T_meas = T_set + (self._T_base-180.0) + random.gauss(0, 0.4)
        v_meas = v_set + (self._v_base-40.0) + random.gauss(0, 0.4)
        return T_meas, v_meas

    def loop(self):
        T_set, v_set, H, per, thr = self._read_inputs()
        T_meas, v_meas = self._simulate_meas(T_set, v_set)
        d = defect_rate_pct(T_meas, v_meas, H)
        y = 100.0 - d
        tp = max(0.0, v_meas)  # items/min 으로 단순화

        self.T_meas_var.set(f"T: {T_meas:.1f} °C")
        self.v_meas_var.set(f"v: {v_meas:.1f} 개/분")
        self.H_var.set(f"H: {H:.1f} %")
        self.def_var.set(f"결함률: {d:.2f} %")
        self.yield_var.set(f"수율: {y:.2f} %")
        self.thr_var.set(f"임계치: {thr:.2f} %")

        alarm = (d >= thr)
        thrthr = 5
        global count
        
        if alarm:
            if count == thrthr:
                self.status_var.set(f"상태: 비상정지(임계치 초과 {thrthr}회)")
                self.stop()
                count = 0
            else:
                self.status_var.set("상태: 경고(결함률 높음)")
                count += 1
        elif not alarm:
            self.status_var.set("상태: 정상")
        
        self.configure(highlightthickness=2, highlightbackground=("#c62828" if alarm else "#2e7d32"))

        if self.log_enabled.get() and self.logger.w:
            self.logger.write([time.strftime("%Y-%m-%d %H:%M:%S"),
                               f"{T_meas:.2f}", f"{v_meas:.2f}", f"{H:.2f}",
                               f"{d:.3f}", f"{y:.3f}", f"{tp:.3f}",
                               (1 if alarm else 0)])

        if self.running:
            self.after_id = self.after(per, self.loop)

    def optimization(self):
        try:
            Tmin = int(self.Tmin_var.get())
            Tmax = int(self.Tmax_var.get())
            Tstep = int(self.Tstep_var.get())
            vmin = int(self.vmin_var.get())
            vmax = int(self.vmax_var.get())
            vstep = int(self.vstep_var.get())
            H = float(self.H_env.get())
            target = float(self.vtarget_var.get())
        except:
            messagebox.showerror("입력 오류", "최적화 파라미터를 확인하세요.")
            return
        min_def, opt_T, opt_v = 100.0, Tmin, vmin
        results = []
        for T in range(Tmin, Tmax+1, Tstep):
            for v in range(vmin, vmax+1, vstep):
                d = defect_rate_pct(T, v, H)
                results.append([T, v, H, d])
                if d < min_def and v>=target:
                    min_def, opt_T, opt_v = d, T, v

        if self.csv_path:
            with open(self.csv_path.replace(".csv","_optimization.csv"),"w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["T","v","H","defect_pct"])
                w.writerows(results)
        self.status_var.set(f"최적 조합[T,v]: {opt_T}, {opt_v} (결함률 {min_def:.2f}%)")
    
    def on_close(self):
        self.stop()
        self.destroy()

if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
