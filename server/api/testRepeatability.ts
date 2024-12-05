export default defineEventHandler(async (event) => {
    console.log("Llamaste a testRepeatability")
    await $fetch('http://localhost:6969', {
        method: 'POST',
        body: {
            command: 'testRepeatability'
        }
    })
    return "Llamaste a testRepeatability"
})
