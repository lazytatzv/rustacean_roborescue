# Contributing

ありがとうございます！このプロジェクトへ貢献する際の簡単な手順とルールを示します。

## 1. 開発フロー
- フォークして新しいブランチを作成してください（例: `feature/xxx`, `fix/yyy`）。
- できるだけ小さなコミットに分け、明確なコミットメッセージを書いてください。

## 2. 必須チェック（PR 前）
- `just fmt` を実行してコードをフォーマットしてください。
- `just check` を実行してローカルのテストと静的解析を走らせてください。
- Python 変更がある場合は `pytest -q` を、Rust 変更がある場合は `cargo test --workspace` を推奨します。

## 3. PR ポリシー
- タイトルに明確な変更点を入れてください（例: `feat: add qr detector node`）。
- 1人以上のレビュワー承認が必要です。重要な変更は squash merge を推奨します。
- 大きな変更（アーキテクチャ、CI、外部依存更新）は事前に issue を立てて議論してください。

## 4. CI とテスト
- PR は自動でフォーマット・linters・ユニットテストが走ります。長時間の統合テストは main ブランチでのみ実行されます。
- ローカルで CI を再現するには Nix を使うか `just` タスクを参照してください（README のクイックスタート参照）。

## 5. スタイルと品質
- Rust: `cargo fmt` / `clippy` を通してから PR を出してください。
- Python: `black` と `ruff`（または `flake8`）を使用してください。

## 6. セキュリティ
- 秘密情報（API keys、証明書、パスワード等）をリポジトリに含めないでください。

## 7. 連絡先
- 質問や大型提案は issue を立ててください。緊急の話題はチームチャットで共有してください。

ありがとうございます — あなたの貢献をお待ちしています！

## 8. VS Code devcontainer の使い方
- このリポジトリは `.devcontainer` を提供しています。VS Code を使用している場合は、コマンドパレットで
	`Remote-Containers: Reopen in Container` を選択すると、開発コンテナ内でプロジェクトを開けます。
- コンテナ作成後、`postCreateCommand` により `nix develop` が実行されます。もしコンテナ内で手動で開始したい場合は:

```bash
# コンテナ内で
nix develop --accept-flake-config
```

- もし VS Code の Remote-Containers 拡張がない場合は、拡張をインストールしてから再試行してください。

## 9. CI が `pre-commit` で失敗した場合の対処

- main ブランチ向けの CI では、`pre-commit` がファイルを変更する場合にビルドを失敗させます。これは自動フォーマットや自動修正がコミットに含まれていることを保証するためです。
- 対処手順:
	1. ローカルで `pre-commit` を実行して自動修正を取り込みます:

```bash
pre-commit run --all-files
git add -A
git commit -m "chore: apply pre-commit fixes"
git push
```

	2. PR が same-repo（フォークでない）であれば、CI は自動で pre-commit の修正をコミットしてプッシュすることがあります。フォークからの PR では権限のため自動プッシュは行われません。
	3. もし自動修正で解決しない場合は、変更点を手動で修正して再度 push してください。

- ヒント: `just check-lint` を使うと、主要なフォーマットと静的解析チェックをローカルでまとめて実行できます。
