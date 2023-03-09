import numpy as np

pseudo_inf = 999999999


def valid_int_input(prompt, upper_bound=pseudo_inf, lower_bound=-pseudo_inf):
    invalid = True
    answer = ""
    while invalid:
        try:
            answer = int(input(prompt))
            if upper_bound > answer > lower_bound:
                invalid = False
        except ValueError:
            pass
    return answer
