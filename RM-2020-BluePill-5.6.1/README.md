# RM2020-Final-Proj-Template

> Note: the content of this template is not complete (intentionally). You need to write your own codes.

## Writing new codes

Work inside the folder `FinalProjCore`, place your codes in the approriate folder (`Communication`, `Driver`, and `Control`).

> Note: You are not forced to follow the project structure, although recommended.

If you add any __NEW FOLDER__ (not new files) inside `FinalProjCore` or inside any folders inside `FinalProjCore`, you need to adjust the Makefile: `final_proj.mk`. You can easily see the patern. If you don't want to get involved with Makefiles, just don't add any new folders.

## Compile

Run `compiledb make -j` to generate `compile_commands.json`, this file allows code completion. 

> Note: if you have not installed compiledb, just run `pip install compiledb` in your terminal.

If you think you don't need code completion, just run `make -j` to compile codes.

## Debug

Install Ozone (a debugger) and upload codes. You should have already done so in Stage 2 assignment. If you are still having trouble uploading codes, __CONSULT SENIORS__.

