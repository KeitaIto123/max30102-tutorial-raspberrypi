import max30102
import hrcalc

m = max30102.MAX30102()

# 100 samples are read and used for HR/SpO2 calculation in a single loop
try:
    while True:
        red, ir = m.read_sequential()
        hr, hr_flag, spo2, spo2_flag =  hrcalc.calc_hr_and_spo2(ir, red)
        print("="*20)
        if hr_flag:
            print(f"心拍数: {hr}")
        else:
            print("心拍数が取れませんでした")

        if spo2_flag:
            print(f"血中酸素濃度: {spo2}")
        else:
            print("血中酸素濃度が取れませんでした")
        print(" ")
except:
    m.shutdown()
