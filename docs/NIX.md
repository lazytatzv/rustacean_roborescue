### 

```bash
git clone --recursive <repourl>
cd rustacean_roborescue

# git submodule update --init --recursive
echo 'direnv hook fish | source' >> ~/.config/fish/config.fish
source ~/.config/fish/config.fish

nix profile install nixpkgs#direnv nixpkgs#nix-direnv
mkdir -p ~/.config/direnv
echo 'source_url "https://raw.githubusercontent.com/nix-community/nix-direnv/master/direnvrc" "sha256-t/qsHuSymixvs+MpxeBGDe0ViFBUrbij5Z9Xy1Bgb28="' > ~/.config/direnv/direnvrc
```

direnv allowが必要


