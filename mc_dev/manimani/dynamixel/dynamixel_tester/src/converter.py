#!/usr/bin/env python3
import sys

class Converter():
    def __init__(self):
        self.ans = 0
        self.deg_or_step = 0
        self.value = 0.0

    def degToStep(self, deg):
        return int((deg+180)/360.0*4095)

    def stepToDeg(self, step):
        return round(step/4095.0*360.0-180, 1)

    def modeChange(self):
        while True:
            try:
                # input
                self.deg_or_step = int(input('1: Degree to Step\n2: Step to Degree\n>>> '))
                print()
                # check
                if self.deg_or_step != 1 and self.deg_or_step != 2:
                    print('Please enter 1 or 2. \n')
                else:
                    break
            except ValueError:
                print('\nPlease enter 1 or 2. \n')
            except KeyboardInterrupt:
                print('\nExit...')
                sys.exit()
       
    def convert(self):
        self.modeChange()
        while True:
            try:
                # convert
                if self.deg_or_step == 1:
                    self.value = float(input('Degree >>> '))
                    self.ans = self.degToStep(self.value)
                else:
                    self.value = float(input('Step >>> '))
                    self.ans = self.stepToDeg(self.value)
                print(f'{self.ans}\n')
            except ValueError:
                print('Please enter a numerical value\n')
            except KeyboardInterrupt:
                print('\n')
                self.modeChange()




if __name__ == '__main__':
    c = Converter()
    c.convert()
