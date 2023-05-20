import tkinter as tk
import threading


class TkApp():

    def startApp(self):
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.i = 0
        self.tk_win = tk.Tk()   

        tk.Label(self.tk_win, text='kp').grid(row=0, column=0)
        self.kp_entry = tk.Entry(self.tk_win); self.kp_entry.grid(row=0, column=1); self.kp_entry.insert(-1, '0')

        tk.Label(self.tk_win, text='ki').grid(row=1, column=0)
        self.ki_entry = tk.Entry(self.tk_win); self.ki_entry.grid(row=1, column=1); self.ki_entry.insert(-1, '0')

        tk.Label(self.tk_win, text='kd').grid(row=2, column=0)
        self.kd_entry = tk.Entry(self.tk_win); self.kd_entry.grid(row=2, column=1); self.kd_entry.insert(-1, '0')

        tk.Button(self.tk_win, text='Reset', command=self.reset_values).grid(row=4, column=0)

        tk.Button(self.tk_win, text='Update', command=self.update_values).grid(row=4, column=1)        

        self.tk_win.mainloop()

    def update_values(self):       
        self.kp = float(self.kp_entry.get())
        self.ki = float(self.ki_entry.get())
        self.kd = float(self.kd_entry.get())
        self.i = 0

    def reset_values(self):       
        self.kp = float(0)
        self.ki = float(0)
        self.kd = float(0)
        self.i = 0
    


if __name__ == "__main__":
    tkApp = TkApp()
    threading.Thread(target=tkApp.startApp).start()
    while True:
        print(f'kp = {tkApp.kp}, ki = {tkApp.ki}, kd = {tkApp.kd}')





