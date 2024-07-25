from enum import Enum

class TextDecorator:
    class Color(Enum):
        RED = '\033[91m'
        GREEN = '\033[92m'
        BLUE = '\033[94m'
        CYAN = '\033[96m'
        WHITE = '\033[97m'
        YELLOW = '\033[93m'
        MAGENTA = '\033[95m'
        GREY = '\033[90m'
        BLACK = '\033[30m'
        ORANGE = '\033[33m'
        ENDC = '\033[0m'

    class Style(Enum):
        BOLD = '\033[1m'
        UNDERLINE = '\033[4m'

    @classmethod
    def decorate(cls, text, color=None, style=None):
        if color:
            if color in cls.Color.__members__:
                text = cls.Color[color].value + text
            else:
                raise ValueError("Color not found in available colors.")
        if style:
            if style in cls.Style.__members__:
                text = cls.Style[style].value + text
            else:
                raise ValueError("Style not found in available styles.")
        return text + cls.Color.ENDC.value

if __name__ == "__main__":
    testo_colorato = TextDecorator.decorate("Testo colorato in rosso", color="RED") + " e testo normale"
    testo_grassetto = TextDecorator.decorate("Testo grassetto", style="BOLD")
    testo_colorato_e_grassetto = TextDecorator.decorate("Testo colorato e grassetto", color="GREEN", style="BOLD")
    print(testo_colorato)
    print(testo_grassetto)
    print(testo_colorato_e_grassetto)
