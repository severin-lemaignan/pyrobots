import logging

def generate_id(with_question_mark = False):
    sequence = "0123456789abcdefghijklmopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
    sample = random.sample(sequence, 5)
    return ("?" + "".join(sample)) if with_question_mark else ("".join(sample))


def logging_setup():
    log_handler = logging.StreamHandler()
    formatter = logging.Formatter("%(message)s")
    log_handler.setFormatter(formatter)
    
    platine_logger = logging.getLogger('module-platine')
    platine_logger.setLevel(logging.DEBUG)
    platine_logger.addHandler(log_handler)
    
    logger = logging.getLogger('supervision')
    logger.setLevel(logging.DEBUG)
    logger.addHandler(log_handler)
    
