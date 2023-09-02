import re
import sys
import os
from black import patched_main


if __name__ == "__main__":
    ws_root = os.environ["SPOT_MARLEY_WS_ROOT"]
    sys.argv[0] = re.sub(r"(-script\.pyw|\.exe)?$", "", sys.argv[0])
    if ws_root != "":
        # yes, appending to sys arg is bad practice but laziness wins
        sys.argv.append(ws_root)
    sys.exit(patched_main())
