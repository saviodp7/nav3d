import tempfile
from collections.abc import Generator
from typing import TypeAlias

import launch
import yaml

YamlValue: TypeAlias = str | int | float | bool


class DictItemReference:
    def __init__(self, dictionary: dict[str, YamlValue], key: str):
        self._dictionary = dictionary
        self._key = key

    def key(self) -> str:
        return self._key

    def setValue(self, value: YamlValue) -> None:
        self._dictionary[self._key] = value


class RewrittenYaml(launch.Substitution):  # type: ignore[misc]
    def __init__(
        self,
        source_file: launch.SomeSubstitutionsType,
        param_rewrites: dict[str, launch.SomeSubstitutionsType],
        root_key: launch.SomeSubstitutionsType | None = None,
        key_rewrites: dict[str, launch.SomeSubstitutionsType] | None = None,
        value_rewrites: dict[str, launch.SomeSubstitutionsType] | None = None,
        convert_types: bool = False,
    ) -> None:
        super().__init__()
        from launch.utilities import normalize_to_list_of_substitutions

        self._source_file = normalize_to_list_of_substitutions(source_file)
        self._param_rewrites = {k: normalize_to_list_of_substitutions(v) for k, v in param_rewrites.items()}
        self._key_rewrites = (
            {k: normalize_to_list_of_substitutions(v) for k, v in key_rewrites.items()} if key_rewrites else {}
        )
        self._value_rewrites = (
            {k: normalize_to_list_of_substitutions(v) for k, v in value_rewrites.items()} if value_rewrites else {}
        )
        self._convert_types = convert_types
        self._root_key = normalize_to_list_of_substitutions(root_key) if root_key else None

    def describe(self) -> str:
        return ""

    def perform(self, context: launch.LaunchContext) -> str:
        source_path = launch.utilities.perform_substitutions(context, self._source_file)
        param_rewrites, key_rewrites, value_rewrites = self._resolve_rewrites(context)

        with open(source_path) as f:
            data = yaml.safe_load(f)

        if data is None:
            data = {}

        self._substitute_params(data, param_rewrites)
        self._add_params(data, param_rewrites)
        self._substitute_keys(data, key_rewrites)
        self._substitute_values(data, value_rewrites)

        if self._root_key is not None:
            rk = launch.utilities.perform_substitutions(context, self._root_key)
            if rk:
                data = {rk: data}

        with tempfile.NamedTemporaryFile(mode="w", delete=False) as out:
            yaml.safe_dump(data, out, default_flow_style=False)
            return out.name

    def _resolve_rewrites(self, context: launch.LaunchContext) -> tuple[dict[str, str], dict[str, str], dict[str, str]]:
        def resolve_map(
            m: dict[str, list[launch.Substitution]],
        ) -> dict[str, str]:
            return {k: launch.utilities.perform_substitutions(context, v) for k, v in m.items()}

        return (
            resolve_map(self._param_rewrites),
            resolve_map(self._key_rewrites),
            resolve_map(self._value_rewrites),
        )

    def _substitute_params(self, node: dict[str, YamlValue], param_rewrites: dict[str, str]) -> None:
        for leaf in self._iter_leaf_keys(node):
            k = leaf.key()
            if k in param_rewrites:
                leaf.setValue(self._convert(param_rewrites[k]))

        yaml_paths = self._pathify(node)
        for path in list(yaml_paths.keys()):
            if path in param_rewrites:
                rewrite_val = self._convert(param_rewrites[path])
                keys = path.split(".")
                self._update_yaml_path_vals(node, keys, rewrite_val)

    def _add_params(self, node: dict[str, YamlValue], param_rewrites: dict[str, str]) -> None:
        yaml_paths = self._pathify(node)
        for path, raw in param_rewrites.items():
            if path in yaml_paths:
                continue
            keys = path.split(".")
            if "ros__parameters" not in keys:
                continue
            self._update_yaml_path_vals(node, keys, self._convert(raw))

    def _substitute_values(self, node: dict[str, YamlValue], value_rewrites: dict[str, str]) -> None:
        def process_value(value):
            if isinstance(value, dict):
                for k, v in list(value.items()):
                    value[k] = process_value(v)
                return value
            if isinstance(value, list):
                return [process_value(v) for v in value]
            sv = str(value)
            if sv in value_rewrites:
                return self._convert(value_rewrites[sv])
            return value

        for k in list(node.keys()):
            node[k] = process_value(node[k])

    def _substitute_keys(self, node: dict[str, YamlValue], key_rewrites: dict[str, str]) -> None:
        if not key_rewrites:
            return
        for k in list(node.keys()):
            v = node[k]
            if k in key_rewrites:
                nk = key_rewrites[k]
                node[nk] = node[k]
                del node[k]
                v = node[nk]
            if isinstance(v, dict):
                self._substitute_keys(v, key_rewrites)

    def _iter_leaf_keys(self, data: dict[str, YamlValue]) -> Generator[DictItemReference, None, None]:
        if not isinstance(data, dict):
            return
        for k in list(data.keys()):
            child = data[k]
            if isinstance(child, dict):
                yield from self._iter_leaf_keys(child)
            yield DictItemReference(data, k)

    def _pathify(
        self,
        d: dict[str, YamlValue] | list[YamlValue] | YamlValue,
        p: str = "",
        paths: dict[str, YamlValue] | None = None,
        joinchar: str = ".",
    ) -> dict[str, YamlValue]:
        if paths is None:
            paths = {}
        pn = (p + joinchar) if p else ""
        if isinstance(d, dict):
            for k, v in d.items():
                self._pathify(v, f"{pn}{k}", paths, joinchar=joinchar)
        elif isinstance(d, list):
            for idx, e in enumerate(d):
                self._pathify(e, f"{pn}{idx}", paths, joinchar=joinchar)
        else:
            paths[p] = d
        return paths

    def _update_yaml_path_vals(
        self,
        node: dict | list,
        keys: list[str],
        rewrite_val: YamlValue,
    ):
        if not keys:
            return node

        head = keys[0]
        tail = keys[1:]

        is_last = len(keys) == 1
        if isinstance(node, list):
            idx = int(head)
            if is_last:
                node[idx] = rewrite_val
                return node
            node[idx] = self._update_yaml_path_vals(node[idx], tail, rewrite_val)
            return node

        if is_last:
            node[head] = rewrite_val
            return node

        nxt = node.get(head)
        if nxt is None:
            node[head] = {}
            nxt = node[head]
        node[head] = self._update_yaml_path_vals(nxt, tail, rewrite_val)
        return node

    def _convert(self, text_value: str) -> YamlValue:
        if self._convert_types:
            try:
                return float(text_value) if "." in text_value else int(text_value)
            except ValueError:
                pass

        tl = text_value.lower()
        if tl == "true":
            return True
        if tl == "false":
            return False

        return text_value
