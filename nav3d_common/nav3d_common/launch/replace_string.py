import tempfile

import launch


class ReplaceString(launch.Substitution):  # type: ignore[misc]
    def __init__(
        self,
        source_file: launch.SomeSubstitutionsType,
        replacements: dict[str, launch.SomeSubstitutionsType],
        condition: launch.Condition | None = None,
    ) -> None:
        super().__init__()
        from launch.utilities import normalize_to_list_of_substitutions

        self._source_file = normalize_to_list_of_substitutions(source_file)
        self._replacements = {k: normalize_to_list_of_substitutions(v) for k, v in replacements.items()}
        self._condition = condition

    def describe(self) -> str:
        return ""

    def perform(self, context: launch.LaunchContext) -> str:
        source_path = launch.utilities.perform_substitutions(context, self._source_file)

        if self._condition is not None and not self._condition.evaluate(context):
            return source_path

        resolved = {k: launch.utilities.perform_substitutions(context, v) for k, v in self._replacements.items()}

        with open(source_path) as in_f, tempfile.NamedTemporaryFile(mode="w", delete=False) as out_f:
            for line in in_f:
                for key, value in resolved.items():
                    line = line.replace(key, value)
                out_f.write(line)
            return out_f.name
