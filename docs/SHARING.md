# Sharing the ONR source deliverable

The ONR deliverable is the Sony ILX-LR1 rig with a Linux/Jetson controller.
Its operator manual is `docs/USER-GUIDE.html`; installation is described in
`docs/INSTALL.md`. Subsequent development for other projects is maintained
separately and is not included in this deliverable.

## Share a self-contained source copy

From the ONR checkout, after the delivery changes have been committed:

```sh
python3 deploy/package-source.py --output ../wildsync-onr-source.zip
```

The command works on Linux or macOS using Python 3.8 or later and Git. It
requires no tracked-file changes and refuses to overwrite an existing output.
Choose a new filename when making another delivery.

Send `wildsync-onr-source.zip` and the SHA256 value printed by the command.
The recipient can check the download before unzipping it:

```sh
# Linux
sha256sum wildsync-onr-source.zip

# macOS
shasum -a 256 wildsync-onr-source.zip
```

The ZIP contains the committed source, the matching field manual and
installation documentation, `LICENSE`, `THIRD_PARTY_NOTICES.md`, and
`BUILD_INFO.json` identifying the exact source commit. It contains no Git
history, local branches, uncommitted files, ignored files, local notes,
recordings, settings or credentials. Keep those private materials outside
tracked source; the exporter includes everything committed to the selected
checkout's HEAD. The recipient needs no access to your local folders or GitHub
account. Unzip it and open `wildsync/docs/USER-GUIDE.html` in a browser.

This is a source delivery, not a configured device image. Sony's proprietary
Camera Remote SDK and compiled binaries are not included. The recipient must
obtain the SDK under Sony's licence and follow `docs/INSTALL.md` to build and
install on the Jetson and camera Pis. Preserve the included licence and
third-party notices when redistributing the source.

## Do not send this working directory directly

A linked Git worktree can contain a `.git` file pointing into another local
checkout. Copying that directory does not create an independent Git repository,
and a broad folder copy can include private local material. Use the ZIP export
above for a source-only delivery.

If the recipient needs Git history instead, share the repository URL or have
them make an independent clone:

```sh
git clone https://github.com/wild-technology/wildsync.git
```

For a reproducible delivery, record the full commit printed by the exporter;
`main` can change later. A release tag can name an agreed delivery snapshot.
